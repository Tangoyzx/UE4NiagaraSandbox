#include "SPH2DSimulatorCPU.h"
#include "UObject/ConstructorHelpers.h"
#include "Engine/Texture2D.h"
#include "Components/ArrowComponent.h"
#include "Components/BillboardComponent.h"
#include "NiagaraComponent.h"
#include "NiagaraSystem.h"
#include "NiagaraFunctionLibrary.h"
#include "NiagaraDataInterfaceArrayFloat.h"

namespace
{
	// UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector()を参考にしている
	void SetNiagaraArrayVector(UNiagaraComponent* NiagaraSystem, FName OverrideName, const TArray<FVector>& ArrayData)
	{
		if (UNiagaraDataInterfaceArrayFloat3* ArrayDI = UNiagaraFunctionLibrary::GetDataInterface<UNiagaraDataInterfaceArrayFloat3>(NiagaraSystem, OverrideName))
		{
			FRWScopeLock WriteLock(ArrayDI->ArrayRWGuard, SLT_Write);
			ArrayDI->FloatData = ArrayData;
			ArrayDI->MarkRenderDataDirty();
		}
	}

	// UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor()を参考にしている
	void SetNiagaraArrayColor(UNiagaraComponent* NiagaraSystem, FName OverrideName, const TArray<FLinearColor>& ArrayData)
	{
		if (UNiagaraDataInterfaceArrayColor* ArrayDI = UNiagaraFunctionLibrary::GetDataInterface<UNiagaraDataInterfaceArrayColor>(NiagaraSystem, OverrideName))
		{
			FRWScopeLock WriteLock(ArrayDI->ArrayRWGuard, SLT_Write);
			ArrayDI->ColorData = ArrayData;
			ArrayDI->MarkRenderDataDirty();
		}
	}
}

void ASPH2DSimulatorCPU::BeginPlay()
{
	Super::BeginPlay();

	Positions.SetNum(NumParticles);
	PrevPositions.SetNum(NumParticles);
	Colors.SetNum(NumParticles);
	Velocities.SetNum(NumParticles);
	Accelerations.SetNum(NumParticles);
	Densities.SetNum(NumParticles);
	Pressures.SetNum(NumParticles);
	Positions3D.SetNum(NumParticles);

	const FVector& ActorWorldLocation = GetActorLocation();
	const FVector2D& ActorWorldLocation2D = FVector2D(ActorWorldLocation.Y, ActorWorldLocation.Z);

	// InitPosRadius半径の円内にランダムに配置
	for (int32 i = 0; i < NumParticles; ++i)
	{
		Positions[i] = ActorWorldLocation2D + WallBox.GetCenter() + FMath::RandPointInCircle(InitPosRadius);
		PrevPositions[i] = Positions[i];
	}

	for (int32 i = 0; i < NumParticles; ++i)
	{
		Colors[i] = FLinearColor(0.0f, 0.7f, 1.0f, 1.0f);
	}

	for (int32 i = 0; i < NumParticles; ++i)
	{
		Velocities[i] = FVector2D::ZeroVector;
	}

	for (int32 i = 0; i < NumParticles; ++i)
	{
		Positions3D[i] = FVector(ActorWorldLocation.X, Positions[i].X, Positions[i].Y);
	}

	if (bUseNeighborGrid3D)
	{
		NeighborGrid3D.Initialize(FIntVector(1, NumCellsX, NumCellsY), MaxNeighborsPerCell);
	}

	// Tick()で設定しても、レベルにNiagaraSystemが最初から配置されていると、初回のスポーンでは配列は初期値を使ってしまい
	//間に合わないのでBeginPlay()でも設定する
	NiagaraComponent->SetNiagaraVariableInt("NumParticles", NumParticles);
	SetNiagaraArrayVector(NiagaraComponent, FName("Positions"), Positions3D);
	SetNiagaraArrayColor(NiagaraComponent, FName("Colors"), Colors);

	DensityCoef = Mass * 4.0f / PI / FMath::Pow(SmoothLength, 8);
	GradientPressureCoef = Mass * -30.0f / PI / FMath::Pow(SmoothLength, 5);
	LaplacianViscosityCoef = Mass * 20.0f / 3.0f / PI / FMath::Pow(SmoothLength, 5);

	SmoothLenSq = SmoothLength * SmoothLength;
	NumThreadParticles = (NumParticles + NumThreads - 1) / NumThreads;
}

void ASPH2DSimulatorCPU::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// アクタ位置の動的な変更に対応し、NeighborGrid3Dへの登録に必要なPostions3DとLocalToUnitTransformを更新しておく
	const FVector& ActorWorldLocation = GetActorLocation();
	for (int32 i = 0; i < NumParticles; ++i)
	{
		Positions3D[i] = FVector(ActorWorldLocation.X, Positions[i].X, Positions[i].Y);
	}

	if (bUseNeighborGrid3D)
	{
		//[-WorldBBoxSize / 2, WorldBBoxSize / 2]を[0,1]に写像して扱う
		LocalToUnitTransform = FTransform(FQuat::Identity, FVector(0.5f), FVector(1.0f) / FVector(1.0f, WorldBBoxSize.X, WorldBBoxSize.Y));
	}

	if (DeltaSeconds > KINDA_SMALL_NUMBER)
	{
		// DeltaSecondsの値の変動に関わらず、シミュレーションに使うサブステップタイムは固定とする
		float SubStepDeltaSeconds = 1.0f / FrameRate / NumIterations;

		for (int32 i = 0; i < NumIterations; ++i)
		{
			Simulate(SubStepDeltaSeconds);
		}
	}

	for (int32 i = 0; i < NumParticles; ++i)
	{
		Positions3D[i] = FVector(ActorWorldLocation.X, Positions[i].X, Positions[i].Y);
	}

	NiagaraComponent->SetNiagaraVariableInt("NumParticles", NumParticles);
	SetNiagaraArrayVector(NiagaraComponent, FName("Positions"), Positions3D);
}

void ASPH2DSimulatorCPU::Simulate(float DeltaSeconds)
{
	for (int32 ParticleIdx = 0; ParticleIdx < NumParticles; ++ParticleIdx)
	{
		Densities[ParticleIdx] = 0.0f;
		Accelerations[ParticleIdx] = FVector2D::ZeroVector;
	}

	if (bUseNeighborGrid3D)
	{
		NeighborGrid3D.Reset();

		// NeighborGrid3Dの構築
		ParallelFor(NumThreads,
			[this](int32 ThreadIndex)
			{
				for (int32 ParticleIdx = NumThreadParticles * ThreadIndex; ParticleIdx < NumThreadParticles * (ThreadIndex + 1) && ParticleIdx < NumParticles; ++ParticleIdx)
				{
					const FVector& UnitPos = NeighborGrid3D.SimulationToUnit(GetActorTransform().InverseTransformPositionNoScale(Positions3D[ParticleIdx]), LocalToUnitTransform);
					const FIntVector& CellIndex = NeighborGrid3D.UnitToIndex(UnitPos);
					if (NeighborGrid3D.IsValidCellIndex(CellIndex))
					{
						int32 LinearIndex = NeighborGrid3D.IndexToLinear(CellIndex);
						int32 PreviousNeighborCount = 0;
						NeighborGrid3D.SetParticleNeighborCount(LinearIndex, 1, PreviousNeighborCount);

						if (PreviousNeighborCount < MaxNeighborsPerCell)
						{
							int32 NeighborGridLinearIndex = NeighborGrid3D.NeighborGridIndexToLinear(CellIndex, PreviousNeighborCount);
							NeighborGrid3D.SetParticleNeighbor(NeighborGridLinearIndex, ParticleIdx);
						}
						else
						{
							UE_LOG(LogTemp, Warning, TEXT("Over registation to NeighborGrid3DCPU. CellIndex=(%d, %d, %d). PreviousNeighborCount=%d."), CellIndex.X, CellIndex.Y, CellIndex.Z, PreviousNeighborCount);
						}
					}
					else
					{
						UE_LOG(LogTemp, Warning, TEXT("There is a particle which is out of NeighborGrid3D. Idx = %d. Position = (%f, %f, %f)."), ParticleIdx, Positions3D[ParticleIdx].X, Positions3D[ParticleIdx].Y, Positions3D[ParticleIdx].Z);
						continue;
					}
				}
			}
		);

		ParallelFor(NumThreads,
			[this](int32 ThreadIndex)
			{
				for (int32 ParticleIdx = NumThreadParticles * ThreadIndex; ParticleIdx < NumThreadParticles * (ThreadIndex + 1) && ParticleIdx < NumParticles; ++ParticleIdx)
				{
					// キャッシュするほどのものでもないのでNeighborGrid3D構築のときと同じ計算をしているのは許容する
					const FVector& UnitPos = NeighborGrid3D.SimulationToUnit(GetActorTransform().InverseTransformPositionNoScale(Positions3D[ParticleIdx]), LocalToUnitTransform);
					const FIntVector& CellIndex = NeighborGrid3D.UnitToIndex(UnitPos);

					if (!NeighborGrid3D.IsValidCellIndex(CellIndex))
					{
						// 構築のときに警告ログを出しているので警告を出すことはしない
						continue;
					}

					static FIntVector AdjacentIndexOffsets[9] = {
						FIntVector(0, -1, -1),
						FIntVector(0, 0, -1),
						FIntVector(0, +1, -1),
						FIntVector(0, -1, 0),
						FIntVector(0, 0, 0),
						FIntVector(0, +1, 0),
						FIntVector(0, -1, +1),
						FIntVector(0, 0, +1),
						FIntVector(0, +1, +1)
					};

					for (int32 AdjIdx = 0; AdjIdx < 9; ++AdjIdx)
					{
						const FIntVector& AdjacentCellIndex = CellIndex + AdjacentIndexOffsets[AdjIdx];
						if (!NeighborGrid3D.IsValidCellIndex(AdjacentCellIndex))
						{
							continue;
						}

						for (int32 NeighborIdx = 0; NeighborIdx < MaxNeighborsPerCell; ++NeighborIdx)
						{
							int32 NeighborLinearIndex = NeighborGrid3D.NeighborGridIndexToLinear(AdjacentCellIndex, NeighborIdx);
							int32 AnotherParticleIdx = NeighborGrid3D.GetParticleNeighbor(NeighborLinearIndex);
							if (ParticleIdx == AnotherParticleIdx || AnotherParticleIdx == INDEX_NONE)
							{
								continue;
							}

							CalculateDensity(ParticleIdx, AnotherParticleIdx);
						}
					}

					CalculatePressure(ParticleIdx);
				}
			}
		);

		// ApplyPressureが他のパーティクルの圧力値を使うので、すべて圧力値を計算してから別ループにする必要がある
		ParallelFor(NumThreads,
			[this, DeltaSeconds](int32 ThreadIndex)
			{
				for (int32 ParticleIdx = NumThreadParticles * ThreadIndex; ParticleIdx < NumThreadParticles * (ThreadIndex + 1) && ParticleIdx < NumParticles; ++ParticleIdx)
				{
					// キャッシュするほどのものでもないのでNeighborGrid3D構築のときと同じ計算をしているのは許容する
					const FVector& UnitPos = NeighborGrid3D.SimulationToUnit(GetActorTransform().InverseTransformPositionNoScale(Positions3D[ParticleIdx]), LocalToUnitTransform);
					const FIntVector& CellIndex = NeighborGrid3D.UnitToIndex(UnitPos);

					if (!NeighborGrid3D.IsValidCellIndex(CellIndex))
					{
						// 構築のときに警告ログを出しているので警告を出すことはしない
						continue;
					}

					static FIntVector AdjacentIndexOffsets[9] = {
						FIntVector(0, -1, -1),
						FIntVector(0, 0, -1),
						FIntVector(0, +1, -1),
						FIntVector(0, -1, 0),
						FIntVector(0, 0, 0),
						FIntVector(0, +1, 0),
						FIntVector(0, -1, +1),
						FIntVector(0, 0, +1),
						FIntVector(0, +1, +1)
					};

					for (int32 AdjIdx = 0; AdjIdx < 9; ++AdjIdx)
					{
						const FIntVector& AdjacentCellIndex = CellIndex + AdjacentIndexOffsets[AdjIdx];
						if (!NeighborGrid3D.IsValidCellIndex(AdjacentCellIndex))
						{
							continue;
						}

						for (int32 NeighborIdx = 0; NeighborIdx < MaxNeighborsPerCell; ++NeighborIdx)
						{
							int32 NeighborLinearIndex = NeighborGrid3D.NeighborGridIndexToLinear(AdjacentCellIndex, NeighborIdx);
							int32 AnotherParticleIdx = NeighborGrid3D.GetParticleNeighbor(NeighborLinearIndex);
							if (ParticleIdx == AnotherParticleIdx || AnotherParticleIdx == INDEX_NONE)
							{
								continue;
							}

							ApplyPressure(ParticleIdx, AnotherParticleIdx);
							ApplyViscosity(ParticleIdx, AnotherParticleIdx, DeltaSeconds);
						}
					}

					if (!bUseWallProjection)
					{
						ApplyWallPenalty(ParticleIdx);
					}
					Integrate(ParticleIdx, DeltaSeconds);
					if (bUseWallProjection)
					{
						ApplyWallProjection(ParticleIdx, DeltaSeconds);
					}
				}
			}
		);
	}
	else
	{
		ParallelFor(NumThreads,
			[this](int32 ThreadIndex)
			{
				for (int32 ParticleIdx = NumThreadParticles * ThreadIndex; ParticleIdx < NumThreadParticles * (ThreadIndex + 1) && ParticleIdx < NumParticles; ++ParticleIdx)
				{
					for (int32 AnotherParticleIdx = 0; AnotherParticleIdx < NumParticles; ++AnotherParticleIdx)
					{
						if (ParticleIdx == AnotherParticleIdx)
						{
							continue;
						}

						CalculateDensity(ParticleIdx, AnotherParticleIdx );
					}

					CalculatePressure(ParticleIdx);
				}
			}
		);

		// ApplyPressureが他のパーティクルの圧力値を使うので、すべて圧力値を計算してから別ループにする必要がある
		ParallelFor(NumThreads,
			[this, DeltaSeconds](int32 ThreadIndex)
			{
				for (int32 ParticleIdx = NumThreadParticles * ThreadIndex; ParticleIdx < NumThreadParticles * (ThreadIndex + 1) && ParticleIdx < NumParticles; ++ParticleIdx)
				{
					for (int32 AnotherParticleIdx = 0; AnotherParticleIdx < NumParticles; ++AnotherParticleIdx)
					{
						if (ParticleIdx == AnotherParticleIdx)
						{
							continue;
						}

						ApplyPressure(ParticleIdx, AnotherParticleIdx);
						ApplyViscosity(ParticleIdx, AnotherParticleIdx, DeltaSeconds);
					}

					if (!bUseWallProjection)
					{
						ApplyWallPenalty(ParticleIdx);
					}
					Integrate(ParticleIdx, DeltaSeconds);
					if (bUseWallProjection)
					{
						ApplyWallProjection(ParticleIdx, DeltaSeconds);
					}
				}
			}
		);
	}
}

void ASPH2DSimulatorCPU::CalculateDensity(int32 ParticleIdx, int32 AnotherParticleIdx)
{
	check(ParticleIdx != AnotherParticleIdx);

	const FVector2D& DiffPos = Positions[AnotherParticleIdx] - Positions[ParticleIdx];
	float DistanceSq = DiffPos.SizeSquared();
	if (DistanceSq < SmoothLenSq)
	{
		float DiffLenSq = SmoothLenSq - DistanceSq;
		Densities[ParticleIdx] += DensityCoef * DiffLenSq * DiffLenSq * DiffLenSq;
	}
}

void ASPH2DSimulatorCPU::CalculatePressure(int32 ParticleIdx)
{
	Pressures[ParticleIdx] = PressureStiffness * FMath::Max(FMath::Pow(Densities[ParticleIdx] / RestDensity, 3) - 1.0f, 0.0f);
}

void ASPH2DSimulatorCPU::ApplyPressure(int32 ParticleIdx, int32 AnotherParticleIdx)
{
	check(ParticleIdx != AnotherParticleIdx);

	if (Densities[ParticleIdx] < SMALL_NUMBER) // 0除算と、小さな値の除算ですごく大きな項になるのを回避
	{
		return;
	}

	const FVector2D& DiffPos = Positions[AnotherParticleIdx] - Positions[ParticleIdx];
	float DistanceSq = DiffPos.SizeSquared();
	float Distance = DiffPos.Size();
	if (DistanceSq < SmoothLenSq
		&& Densities[AnotherParticleIdx] > SMALL_NUMBER && Distance > SMALL_NUMBER) // 0除算と、小さな値の除算ですごく大きな項になるのを回避
	{
		float DiffLen = SmoothLength - Distance;
#if 1
		// 数式と違うが、UnityGraphicsProgramming1がソースコードで使っていた式。こちらの方がなぜか安定するしフレームレートも上がる
		float AvgPressure = 0.5f * (Pressures[ParticleIdx] + Pressures[AnotherParticleIdx]);
		const FVector2D& Pressure = GradientPressureCoef * AvgPressure / Densities[AnotherParticleIdx] * DiffLen * DiffLen / Distance * DiffPos;
#else
		float DiffPressure = Pressures[ParticleIdx] - Pressures[AnotherParticleIdx];
		const FVector2D& Pressure = GradientPressureCoef * DiffPressure / Densities[AnotherParticleIdx] * DiffLen * DiffLen / Distance * DiffPos;
#endif

		Accelerations[ParticleIdx] += Pressure / Densities[ParticleIdx];
	}
}

void ASPH2DSimulatorCPU::ApplyViscosity(int32 ParticleIdx, int32 AnotherParticleIdx, float DeltaSeconds)
{
	check(ParticleIdx != AnotherParticleIdx);

	if (Densities[ParticleIdx] < SMALL_NUMBER) // 0除算と、小さな値の除算ですごく大きな項になるのを回避
	{
		return;
	}

	const FVector2D& DiffPos = Positions[AnotherParticleIdx] - Positions[ParticleIdx];
	float DistanceSq = DiffPos.SizeSquared();
	if (DistanceSq < SmoothLenSq
		&& Densities[AnotherParticleIdx] > SMALL_NUMBER) // 0除算と、小さな値の除算ですごく大きな項になるのを回避
	{
		FVector2D DiffVel;
		if (bUseWallProjection)
		{
			DiffVel = ((Positions[AnotherParticleIdx] - PrevPositions[AnotherParticleIdx]) - (Positions[ParticleIdx] - PrevPositions[ParticleIdx])) / DeltaSeconds;
		}
		else
		{
			DiffVel = Velocities[AnotherParticleIdx] - Velocities[ParticleIdx];
		}
		const FVector2D& ViscosityForce = LaplacianViscosityCoef / Densities[AnotherParticleIdx] * (SmoothLength - DiffPos.Size()) * DiffVel;
		Accelerations[ParticleIdx] += Viscosity * ViscosityForce / Densities[ParticleIdx];
	}
}

void ASPH2DSimulatorCPU::ApplyWallPenalty(int32 ParticleIdx)
{
	// 計算が楽なので、アクタの位置移動と回転を戻した座標系でパーティクル位置を扱う
	const FVector& Position3D = FVector(GetActorLocation().X, Positions[ParticleIdx].X, Positions[ParticleIdx].Y);
	const FVector& InvActorMovePos = GetActorTransform().InverseTransformPositionNoScale(Position3D);

	// 上境界
	FVector TopAccel = FMath::Max(0.0f, InvActorMovePos.Z - WallBox.Max.Y) * WallStiffness * FVector(0.0f, 0.0f, -1.0f);
	TopAccel = GetActorTransform().TransformVectorNoScale(TopAccel);
	Accelerations[ParticleIdx] += FVector2D(TopAccel.Y, TopAccel.Z);
	// 下境界
	FVector BottomAccel = FMath::Max(0.0f, WallBox.Min.Y - InvActorMovePos.Z) * WallStiffness * FVector(0.0f, 0.0f, 1.0f);
	BottomAccel = GetActorTransform().TransformVectorNoScale(BottomAccel);
	Accelerations[ParticleIdx] += FVector2D(BottomAccel.Y, BottomAccel.Z);
	// 左境界
	FVector LeftAccel = FMath::Max(0.0f, WallBox.Min.X - InvActorMovePos.Y) * WallStiffness * FVector(0.0f, 1.0f, 0.0f);
	LeftAccel = GetActorTransform().TransformVectorNoScale(LeftAccel);
	Accelerations[ParticleIdx] += FVector2D(LeftAccel.Y, LeftAccel.Z);
	// 右境界
	FVector RightAccel = FMath::Max(0.0f, InvActorMovePos.Y - WallBox.Max.X) * WallStiffness * FVector(0.0f, -1.0f, 0.0f);
	RightAccel = GetActorTransform().TransformVectorNoScale(RightAccel);
	Accelerations[ParticleIdx] += FVector2D(RightAccel.Y, RightAccel.Z);
}

void ASPH2DSimulatorCPU::Integrate(int32 ParticleIdx, float DeltaSeconds)
{
	Accelerations[ParticleIdx] += FVector2D(0.0f, Gravity);
	if (bUseWallProjection)
	{
		const FVector2D& NewPosition = Positions[ParticleIdx] + (Positions[ParticleIdx] - PrevPositions[ParticleIdx])+ Accelerations[ParticleIdx] * DeltaSeconds * DeltaSeconds;
		PrevPositions[ParticleIdx] = Positions[ParticleIdx];
		Positions[ParticleIdx] = NewPosition;
	}
	else
	{
		// 前進オイラー法
		Velocities[ParticleIdx] += Accelerations[ParticleIdx] * DeltaSeconds;

		// MaxVelocityによるクランプ
		FVector2D VelocityNormalized;
		float Velocity;
		Velocities[ParticleIdx].ToDirectionAndLength(VelocityNormalized, Velocity);
		if (Velocity > MaxVelocity)
		{
			Velocities[ParticleIdx] = VelocityNormalized * MaxVelocity;
		}

		Positions[ParticleIdx] += Velocities[ParticleIdx] * DeltaSeconds;
	}
}

void ASPH2DSimulatorCPU::ApplyWallProjection(int32 ParticleIdx, float DeltaSeconds)
{
	// 計算が楽なので、アクタの位置移動と回転を戻した座標系でパーティクル位置を扱う
	// 壁の法線方向を使った内積を使うやり方もあるが
	const FVector& Position3D = FVector(GetActorLocation().X, Positions[ParticleIdx].X, Positions[ParticleIdx].Y);
	const FVector& InvActorMovePos = GetActorTransform().InverseTransformPositionNoScale(Position3D);

	//TODO: WallProjectionAlphaによる効果はNumIterationの影響が大きい。可変フレームレート対応もできてない
	FVector ProjectedPos = InvActorMovePos;
	ProjectedPos += FMath::Max(0.0f, InvActorMovePos.Z - WallBox.Max.Y) * FVector(0.0f, 0.0f, -1.0f) * WallProjectionAlpha;
	ProjectedPos += FMath::Max(0.0f, WallBox.Min.Y - InvActorMovePos.Z) * FVector(0.0f, 0.0f, 1.0f) * WallProjectionAlpha;
	ProjectedPos += FMath::Max(0.0f, WallBox.Min.X - InvActorMovePos.Y) * FVector(0.0f, 1.0f, 0.0f) * WallProjectionAlpha;
	ProjectedPos += FMath::Max(0.0f, InvActorMovePos.Y - WallBox.Max.X) * FVector(0.0f, -1.0f, 0.0f) * WallProjectionAlpha;
	ProjectedPos = GetActorTransform().TransformPositionNoScale(ProjectedPos);

	Positions[ParticleIdx] = FVector2D(ProjectedPos.Y, ProjectedPos.Z);

	// MaxVelocityによるクランプ
	FVector2D VelocityNormalized;
	float Velocity;
	((Positions[ParticleIdx] - PrevPositions[ParticleIdx]) / DeltaSeconds).ToDirectionAndLength(VelocityNormalized, Velocity);
	if (Velocity > MaxVelocity)
	{
		Positions[ParticleIdx] = VelocityNormalized * MaxVelocity * DeltaSeconds + PrevPositions[ParticleIdx];
	}
}

ASPH2DSimulatorCPU::ASPH2DSimulatorCPU()
{
	PrimaryActorTick.bCanEverTick = true;

	NiagaraComponent = CreateDefaultSubobject<UNiagaraComponent>(TEXT("NiagaraComponent0"));

	RootComponent = NiagaraComponent;

#if WITH_EDITORONLY_DATA
	SpriteComponent = CreateEditorOnlyDefaultSubobject<UBillboardComponent>(TEXT("Sprite"));
	ArrowComponent = CreateEditorOnlyDefaultSubobject<UArrowComponent>(TEXT("ArrowComponent0"));

	if (!IsRunningCommandlet())
	{
		// Structure to hold one-time initialization
		struct FConstructorStatics
		{
			ConstructorHelpers::FObjectFinderOptional<UTexture2D> SpriteTextureObject;
			FName ID_Effects;
			FText NAME_Effects;
			FConstructorStatics()
				: SpriteTextureObject(TEXT("/Niagara/Icons/S_ParticleSystem"))
				, ID_Effects(TEXT("Effects"))
				, NAME_Effects(NSLOCTEXT("SpriteCategory", "Effects", "Effects"))
			{
			}
		};
		static FConstructorStatics ConstructorStatics;

		if (SpriteComponent)
		{
			SpriteComponent->Sprite = ConstructorStatics.SpriteTextureObject.Get();
			SpriteComponent->SetRelativeScale3D_Direct(FVector(0.5f, 0.5f, 0.5f));
			SpriteComponent->bHiddenInGame = true;
			SpriteComponent->bIsScreenSizeScaled = true;
			SpriteComponent->SpriteInfo.Category = ConstructorStatics.ID_Effects;
			SpriteComponent->SpriteInfo.DisplayName = ConstructorStatics.NAME_Effects;
			SpriteComponent->SetupAttachment(NiagaraComponent);
			SpriteComponent->bReceivesDecals = false;
		}

		if (ArrowComponent)
		{
			ArrowComponent->ArrowColor = FColor(0, 255, 128);

			ArrowComponent->ArrowSize = 1.5f;
			ArrowComponent->bTreatAsASprite = true;
			ArrowComponent->bIsScreenSizeScaled = true;
			ArrowComponent->SpriteInfo.Category = ConstructorStatics.ID_Effects;
			ArrowComponent->SpriteInfo.DisplayName = ConstructorStatics.NAME_Effects;
			ArrowComponent->SetupAttachment(NiagaraComponent);
			ArrowComponent->SetUsingAbsoluteScale(true);
		}
	}
#endif // WITH_EDITORONLY_DATA
}

void ASPH2DSimulatorCPU::PostRegisterAllComponents()
{
	Super::PostRegisterAllComponents();

	// Set Notification Delegate
	if (NiagaraComponent)
	{
		NiagaraComponent->OnSystemFinished.AddUniqueDynamic(this, &ASPH2DSimulatorCPU::OnNiagaraSystemFinished);
	}
}

void ASPH2DSimulatorCPU::SetDestroyOnSystemFinish(bool bShouldDestroyOnSystemFinish)
{
	bDestroyOnSystemFinish = bShouldDestroyOnSystemFinish ? 1 : 0;  
};

void ASPH2DSimulatorCPU::OnNiagaraSystemFinished(UNiagaraComponent* FinishedComponent)
{
	if (bDestroyOnSystemFinish)
	{
		SetLifeSpan(0.0001f);
	}
}

#if WITH_EDITOR
bool ASPH2DSimulatorCPU::GetReferencedContentObjects(TArray<UObject*>& Objects) const
{
	Super::GetReferencedContentObjects(Objects);

	if (UNiagaraSystem* System = NiagaraComponent->GetAsset())
	{
		Objects.Add(System);
	}

	return true;
}

void ASPH2DSimulatorCPU::ResetInLevel()
{
	if (NiagaraComponent)
	{
		NiagaraComponent->Activate(true);
		NiagaraComponent->ReregisterComponent();
	}
}
#endif // WITH_EDITOR

