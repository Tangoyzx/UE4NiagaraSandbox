#include "SPH3DSimulatorCPU.h"
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
	FVector RandPointInSphere(const FBoxSphereBounds& BoxSphere)
	{
		FVector Point;
		float L;

		do
		{
			Point = FMath::RandPointInBox(BoxSphere.GetBox());
			L = Point.SizeSquared();
		}
		while (L > BoxSphere.SphereRadius);

		return Point;
	}

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

void ASPH3DSimulatorCPU::BeginPlay()
{
	Super::BeginPlay();

	Positions.SetNum(NumParticles);
	Colors.SetNum(NumParticles);
	Velocities.SetNum(NumParticles);
	Accelerations.SetNum(NumParticles);
	Densities.SetNum(NumParticles);
	Pressures.SetNum(NumParticles);

	// InitPosRadius半径の球内にランダムに配置
	FBoxSphereBounds BoxSphere(WallBox.GetCenter(), FVector(InitPosRadius), InitPosRadius);
	for (int32 i = 0; i < NumParticles; ++i)
	{
		Positions[i] = RandPointInSphere(WallBox);
	}

	for (int32 i = 0; i < NumParticles; ++i)
	{
		Colors[i] = FLinearColor(0.0f, 0.7f, 1.0f, 1.0f);
	}

	for (int32 i = 0; i < NumParticles; ++i)
	{
		Velocities[i] = FVector::ZeroVector;
	}

	if (bUseNeighborGrid3D)
	{
		//TODO: [-WorldBBoxSize / 2, WorldBBoxSize / 2]を[0,1]に写像して扱う。RotationとTranslationはなし
		SimulationToUnitTransform = FTransform(FQuat::Identity, FVector(0.5f), FVector(1.0f) / WorldBBoxSize);
		NeighborGrid3D.Initialize(FIntVector(NumCellsX, NumCellsY, NumCellsZ), MaxNeighborsPerCell);
	}

	// Tick()で設定しても、レベルにNiagaraSystemが最初から配置されていると、初回のスポーンでは配列は初期値を使ってしまい
	//間に合わないのでBeginPlay()でも設定する
	NiagaraComponent->SetNiagaraVariableInt("NumParticles", NumParticles);
	SetNiagaraArrayVector(NiagaraComponent, FName("Positions"), Positions);
	SetNiagaraArrayColor(NiagaraComponent, FName("Colors"), Colors);

	DensityCoef = Mass * 4.0f / PI / FMath::Pow(SmoothLength, 8);
	GradientPressureCoef = Mass * -30.0f / PI / FMath::Pow(SmoothLength, 5);
	LaplacianViscosityCoef = Mass * 20.0f / 3.0f / PI / FMath::Pow(SmoothLength, 5);

	SmoothLenSq = SmoothLength * SmoothLength;
	NumThreadParticles = (NumParticles + NumThreads - 1) / NumThreads;
}

void ASPH3DSimulatorCPU::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (DeltaSeconds > KINDA_SMALL_NUMBER)
	{
		// DeltaSecondsの値の変動に関わらず、シミュレーションに使うサブステップタイムは固定とする
		float SubStepDeltaSeconds = 1.0f / FrameRate / NumIterations;

		for (int32 i = 0; i < NumIterations; ++i)
		{
			Simulate(SubStepDeltaSeconds);
		}
	}

	NiagaraComponent->SetNiagaraVariableInt("NumParticles", NumParticles);
	SetNiagaraArrayVector(NiagaraComponent, FName("Positions"), Positions);
}

void ASPH3DSimulatorCPU::Simulate(float DeltaSeconds)
{
	for (int32 ParticleIdx = 0; ParticleIdx < NumParticles; ++ParticleIdx)
	{
		Densities[ParticleIdx] = 0.0f;
		Accelerations[ParticleIdx] = FVector::ZeroVector;
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
					const FVector& UnitPos = NeighborGrid3D.SimulationToUnit(Positions[ParticleIdx], SimulationToUnitTransform);
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
						UE_LOG(LogTemp, Warning, TEXT("There is a particle which is out of NeighborGrid3D. Idx = %d. Position = (%f, %f, %f)."), ParticleIdx, Positions[ParticleIdx].X, Positions[ParticleIdx].Y, Positions[ParticleIdx].Z);
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
					const FVector& UnitPos = NeighborGrid3D.SimulationToUnit(Positions[ParticleIdx], SimulationToUnitTransform);
					const FIntVector& CellIndex = NeighborGrid3D.UnitToIndex(UnitPos);

					if (!NeighborGrid3D.IsValidCellIndex(CellIndex))
					{
						// 構築のときに警告ログを出しているので警告を出すことはしない
						continue;
					}

					static FIntVector AdjacentIndexOffsets[27] = {
						FIntVector(-1, -1, -1),
						FIntVector(-1, 0, -1),
						FIntVector(-1, +1, -1),
						FIntVector(-1, -1, 0),
						FIntVector(-1, 0, 0),
						FIntVector(-1, +1, 0),
						FIntVector(-1, -1, +1),
						FIntVector(-1, 0, +1),
						FIntVector(-1, +1, +1),

						FIntVector(0, -1, -1),
						FIntVector(0, 0, -1),
						FIntVector(0, +1, -1),
						FIntVector(0, -1, 0),
						FIntVector(0, 0, 0),
						FIntVector(0, +1, 0),
						FIntVector(0, -1, +1),
						FIntVector(0, 0, +1),
						FIntVector(0, +1, +1),

						FIntVector(+1, -1, -1),
						FIntVector(+1, 0, -1),
						FIntVector(+1, +1, -1),
						FIntVector(+1, -1, 0),
						FIntVector(+1, 0, 0),
						FIntVector(+1, +1, 0),
						FIntVector(+1, -1, +1),
						FIntVector(+1, 0, +1),
						FIntVector(+1, +1, +1),
					};

					for (int32 AdjIdx = 0; AdjIdx < 27; ++AdjIdx)
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
					const FVector& UnitPos = NeighborGrid3D.SimulationToUnit(Positions[ParticleIdx], SimulationToUnitTransform);
					const FIntVector& CellIndex = NeighborGrid3D.UnitToIndex(UnitPos);

					if (!NeighborGrid3D.IsValidCellIndex(CellIndex))
					{
						// 構築のときに警告ログを出しているので警告を出すことはしない
						continue;
					}

					static FIntVector AdjacentIndexOffsets[27] = {
						FIntVector(-1, -1, -1),
						FIntVector(-1, 0, -1),
						FIntVector(-1, +1, -1),
						FIntVector(-1, -1, 0),
						FIntVector(-1, 0, 0),
						FIntVector(-1, +1, 0),
						FIntVector(-1, -1, +1),
						FIntVector(-1, 0, +1),
						FIntVector(-1, +1, +1),

						FIntVector(0, -1, -1),
						FIntVector(0, 0, -1),
						FIntVector(0, +1, -1),
						FIntVector(0, -1, 0),
						FIntVector(0, 0, 0),
						FIntVector(0, +1, 0),
						FIntVector(0, -1, +1),
						FIntVector(0, 0, +1),
						FIntVector(0, +1, +1),

						FIntVector(+1, -1, -1),
						FIntVector(+1, 0, -1),
						FIntVector(+1, +1, -1),
						FIntVector(+1, -1, 0),
						FIntVector(+1, 0, 0),
						FIntVector(+1, +1, 0),
						FIntVector(+1, -1, +1),
						FIntVector(+1, 0, +1),
						FIntVector(+1, +1, +1),
					};

					for (int32 AdjIdx = 0; AdjIdx < 27; ++AdjIdx)
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
							ApplyViscosity(ParticleIdx, AnotherParticleIdx);
						}
					}

					ApplyWallPenalty(ParticleIdx);
					Integrate(ParticleIdx, DeltaSeconds);
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
						ApplyViscosity(ParticleIdx, AnotherParticleIdx);
					}

					ApplyWallPenalty(ParticleIdx);
					Integrate(ParticleIdx, DeltaSeconds);
				}
			}
		);
	}
}

void ASPH3DSimulatorCPU::CalculateDensity(int32 ParticleIdx, int32 AnotherParticleIdx)
{
	check(ParticleIdx != AnotherParticleIdx);

	const FVector& DiffPos = Positions[AnotherParticleIdx] - Positions[ParticleIdx];
	float DistanceSq = DiffPos.SizeSquared();
	if (DistanceSq < SmoothLenSq)
	{
		float DiffLenSq = SmoothLenSq - DistanceSq;
		Densities[ParticleIdx] += DensityCoef * DiffLenSq * DiffLenSq * DiffLenSq;
	}
}

void ASPH3DSimulatorCPU::CalculatePressure(int32 ParticleIdx)
{
	Pressures[ParticleIdx] = PressureStiffness * FMath::Max(FMath::Pow(Densities[ParticleIdx] / RestDensity, 7) - 1.0f, 0.0f);
}

void ASPH3DSimulatorCPU::ApplyPressure(int32 ParticleIdx, int32 AnotherParticleIdx)
{
	check(ParticleIdx != AnotherParticleIdx);

	if (Densities[ParticleIdx] < SMALL_NUMBER) // 0除算と、小さな値の除算ですごく大きな項になるのを回避
	{
		return;
	}

	const FVector& DiffPos = Positions[AnotherParticleIdx] - Positions[ParticleIdx];
	float DistanceSq = DiffPos.SizeSquared();
	float Distance = DiffPos.Size();
	if (DistanceSq < SmoothLenSq
		&& Densities[AnotherParticleIdx] > SMALL_NUMBER && Distance > SMALL_NUMBER) // 0除算と、小さな値の除算ですごく大きな項になるのを回避
	{
		float DiffLen = SmoothLength - Distance;
#if 1
		// 数式と違うが、UnityGraphicsProgramming1がソースコードで使っていた式。こちらの方がなぜか安定するしフレームレートも上がる
		float AvgPressure = 0.5f * (Pressures[ParticleIdx] + Pressures[AnotherParticleIdx]);
		const FVector& Pressure = GradientPressureCoef * AvgPressure / Densities[AnotherParticleIdx] * DiffLen * DiffLen / Distance * DiffPos;
#else
		float DiffPressure = Pressures[ParticleIdx] - Pressures[AnotherParticleIdx];
		const FVector& Pressure = GradientPressureCoef * DiffPressure / Densities[AnotherParticleIdx] * DiffLen * DiffLen / Distance * DiffPos;
#endif

		Accelerations[ParticleIdx] += Pressure / Densities[ParticleIdx];
	}
}

void ASPH3DSimulatorCPU::ApplyViscosity(int32 ParticleIdx, int32 AnotherParticleIdx)
{
	check(ParticleIdx != AnotherParticleIdx);

	if (Densities[ParticleIdx] < SMALL_NUMBER) // 0除算と、小さな値の除算ですごく大きな項になるのを回避
	{
		return;
	}

	const FVector& DiffPos = Positions[AnotherParticleIdx] - Positions[ParticleIdx];
	float DistanceSq = DiffPos.SizeSquared();
	if (DistanceSq < SmoothLenSq
		&& Densities[AnotherParticleIdx] > SMALL_NUMBER) // 0除算と、小さな値の除算ですごく大きな項になるのを回避
	{
		const FVector& DiffVel = Velocities[AnotherParticleIdx] - Velocities[ParticleIdx];
		const FVector& ViscosityForce = LaplacianViscosityCoef / Densities[AnotherParticleIdx] * (SmoothLength - DiffPos.Size()) * DiffVel;
		Accelerations[ParticleIdx] += Viscosity * ViscosityForce / Densities[ParticleIdx];
	}
}

void ASPH3DSimulatorCPU::ApplyWallPenalty(int32 ParticleIdx)
{
	//TODO: SPHって言っても加速度使わずにPBD使ってもいいはずなんだよな
	// 上境界
	Accelerations[ParticleIdx] += FMath::Max(0.0f, Positions[ParticleIdx].Z - WallBox.Max.Z) * WallStiffness * FVector(0.0f, 0.0f, -1.0f);
	// 下境界
	Accelerations[ParticleIdx] += FMath::Max(0.0f, WallBox.Min.Z - Positions[ParticleIdx].Z) * WallStiffness * FVector(0.0f, 0.0f, 1.0f);
	// 左境界
	Accelerations[ParticleIdx] += FMath::Max(0.0f, WallBox.Min.X - Positions[ParticleIdx].X) * WallStiffness * FVector(1.0f, 0.0f, 0.0f);
	// 右境界
	Accelerations[ParticleIdx] += FMath::Max(0.0f, Positions[ParticleIdx].X - WallBox.Max.X) * WallStiffness * FVector(-1.0f, 0.0f, 0.0f);
	// 奥境界
	Accelerations[ParticleIdx] += FMath::Max(0.0f, WallBox.Min.Y - Positions[ParticleIdx].Y) * WallStiffness * FVector(0.0f, 1.0f, 0.0f);
	// 手前境界
	Accelerations[ParticleIdx] += FMath::Max(0.0f, Positions[ParticleIdx].Y - WallBox.Max.Y) * WallStiffness * FVector(0.0f, -1.0f, 0.0f);
}

void ASPH3DSimulatorCPU::Integrate(int32 ParticleIdx, float DeltaSeconds)
{
	Accelerations[ParticleIdx] += FVector(0.0f, 0.0f, Gravity);
	// 前進オイラー法
	Velocities[ParticleIdx] += Accelerations[ParticleIdx] * DeltaSeconds;
	Positions[ParticleIdx] += Velocities[ParticleIdx] * DeltaSeconds;
}

ASPH3DSimulatorCPU::ASPH3DSimulatorCPU()
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

void ASPH3DSimulatorCPU::PostRegisterAllComponents()
{
	Super::PostRegisterAllComponents();

	// Set Notification Delegate
	if (NiagaraComponent)
	{
		NiagaraComponent->OnSystemFinished.AddUniqueDynamic(this, &ASPH3DSimulatorCPU::OnNiagaraSystemFinished);
	}
}

void ASPH3DSimulatorCPU::SetDestroyOnSystemFinish(bool bShouldDestroyOnSystemFinish)
{
	bDestroyOnSystemFinish = bShouldDestroyOnSystemFinish ? 1 : 0;  
};

void ASPH3DSimulatorCPU::OnNiagaraSystemFinished(UNiagaraComponent* FinishedComponent)
{
	if (bDestroyOnSystemFinish)
	{
		SetLifeSpan(0.0001f);
	}
}

#if WITH_EDITOR
bool ASPH3DSimulatorCPU::GetReferencedContentObjects(TArray<UObject*>& Objects) const
{
	Super::GetReferencedContentObjects(Objects);

	if (UNiagaraSystem* System = NiagaraComponent->GetAsset())
	{
		Objects.Add(System);
	}

	return true;
}

void ASPH3DSimulatorCPU::ResetInLevel()
{
	if (NiagaraComponent)
	{
		NiagaraComponent->Activate(true);
		NiagaraComponent->ReregisterComponent();
	}
}
#endif // WITH_EDITOR

