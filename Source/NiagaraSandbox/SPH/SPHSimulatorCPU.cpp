#include "SPHSimulatorCPU.h"
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
}

void ASPHSimulatorCPU::BeginPlay()
{
	Super::BeginPlay();

	Positions.SetNum(NumParticles);
	Velocities.SetNum(NumParticles);
	Accelerations.SetNum(NumParticles);
	Densities.SetNum(NumParticles);
	Pressures.SetNum(NumParticles);
	Positions3D.SetNum(NumParticles);

	// InitPosRadius半径の円内にランダムに配置
	for (int32 i = 0; i < NumParticles; ++i)
	{
		Positions[i] = WallBox.GetCenter() + FMath::RandPointInCircle(InitPosRadius);
	}

	for (int32 i = 0; i < NumParticles; ++i)
	{
		Velocities[i] = FVector2D::ZeroVector;
	}

	for (int32 i = 0; i < NumParticles; ++i)
	{
		Positions3D[i] = FVector(0.0f, Positions[i].X, Positions[i].Y);
	}

	if (bUseNeighborGrid3D)
	{
		//TODO: [-WorldBBoxSize / 2, WorldBBoxSize / 2]を[0,1]に写像して扱う。RotationとTranslationはなし
		SimulationToUnitTransform = FTransform(FQuat::Identity, FVector(0.5f), FVector(1.0f) / WorldBBoxSize);
		NeighborGrid3D.Initialize(NumCells, MaxNeighborsPerCell);
	}

	// Tick()で設定しても、レベルにNiagaraSystemが最初から配置されていると、初回のスポーンでは配列は初期値を使ってしまい
	//間に合わないのでBeginPlay()でも設定する
	NiagaraComponent->SetNiagaraVariableInt("NumParticles", NumParticles);
	SetNiagaraArrayVector(NiagaraComponent, FName("Positions"), Positions3D);

	DensityCoef = Mass * 4.0f / PI / FMath::Pow(SmoothLength, 8);
	GradientPressureCoef = Mass * -30.0f / PI / FMath::Pow(SmoothLength, 5);
	LaplacianViscosityCoef = Mass * 20.0f / 3.0f / PI / FMath::Pow(SmoothLength, 5);

	SmoothLenSq = SmoothLength * SmoothLength;
	NumThreadParticles = (NumParticles + NumThreads - 1) / NumThreads;
}

void ASPHSimulatorCPU::Tick(float DeltaSeconds)
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

	for (int32 i = 0; i < NumParticles; ++i)
	{
		Positions3D[i] = FVector(0.0f, Positions[i].X, Positions[i].Y);
	}

	NiagaraComponent->SetNiagaraVariableInt("NumParticles", NumParticles);
	SetNiagaraArrayVector(NiagaraComponent, FName("Positions"), Positions3D);
}

void ASPHSimulatorCPU::Simulate(float DeltaSeconds)
{
	if (bUseNeighborGrid3D)
	{
		NeighborGrid3D.Reset();

		for (int32 ParticleIdx = 0; ParticleIdx < NumParticles; ++ParticleIdx)
		{
			const FVector& UnitPos = NeighborGrid3D.SimulationToUnit(Positions3D[ParticleIdx], SimulationToUnitTransform);
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
		}
	}

	// TODO:初期化はあとで加速度を計算するようになったら不要になるのであとで消す
	for (int32 ParticleIdx = 0; ParticleIdx < NumParticles; ++ParticleIdx)
	{
		Accelerations[ParticleIdx] = FVector2D::ZeroVector;
	}

	// TODO:それぞれのフェイズでParallerForしてるが、そもそも全部まとめてParallelForすることもできるはず
	// CSと違って一つのタスクを小さくするのがいいわけでもないので
	CalculateDensity();
	CalculatePressure();
	ApplyPressure();
	ApplyViscosity();
	ApplyWallPenalty();
	Integrate(DeltaSeconds);
}

void ASPHSimulatorCPU::CalculateDensity()
{
#if 1
	for (int32 i = 0; i < NumParticles; ++i)
	{
		Densities[i] = 0.0f;

		#if 1
		// TODO:各フェイズで毎回計算するのは冗長
		const FVector& UnitPos = NeighborGrid3D.SimulationToUnit(Positions3D[i], SimulationToUnitTransform);
		const FIntVector& CellIndex = NeighborGrid3D.UnitToIndex(UnitPos);
		// TODO:判定条件も共通関数化できるな
		if (!NeighborGrid3D.IsValidCellIndex(CellIndex))
		{
			// TODO:はみ出るのはどういうケースだ？
			continue;
		}

		FIntVector AdjacentIndexOffsets[9] = {
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

		for (int32 j = 0; j < 9; ++j)
		{
			const FIntVector& AdjacentCellIndex = CellIndex + AdjacentIndexOffsets[j];
			// TODO:判定条件も共通関数化できるな
			if (!NeighborGrid3D.IsValidCellIndex(AdjacentCellIndex))
			{
				continue;
			}

			for (int32 k = 0; k < MaxNeighborsPerCell; ++k)
			{
				int32 NeighborLinearIndex = NeighborGrid3D.NeighborGridIndexToLinear(AdjacentCellIndex, k);
				int32 ParticleIndex = NeighborGrid3D.GetParticleNeighbor(NeighborLinearIndex);
				if (i != ParticleIndex && ParticleIndex != INDEX_NONE)
				{
					const FVector2D& DiffPos = Positions[ParticleIndex] - Positions[i];
					float DistanceSq = DiffPos.SizeSquared();
					if (DistanceSq < SmoothLenSq)
					{
						float DiffLenSq = SmoothLenSq - DistanceSq;
						Densities[i] += DensityCoef * DiffLenSq * DiffLenSq * DiffLenSq;
					}
				}
			}
		}
		#else
		for (int32 j = 0; j < NumParticles; ++j)
		{
			if (i == j)
			{
				continue;
			}

			const FVector2D& DiffPos = Positions[j] - Positions[i];
			float DistanceSq = DiffPos.SizeSquared();
			if (DistanceSq < SmoothLenSq)
			{
				float DiffLenSq = SmoothLenSq - DistanceSq;
				Densities[i] += DensityCoef * DiffLenSq * DiffLenSq * DiffLenSq;
			}
		}
		#endif
	}
#else
	ParallelFor(NumThreads,
		[this](int32 ThreadIndex)
		{
			for (int32 i = NumThreadParticles * ThreadIndex; i < NumThreadParticles * (ThreadIndex + 1) && i < NumParticles; ++i)
			{
				Densities[i] = 0.0f;

				for (int32 j = 0; j < NumParticles; ++j)
				{
					if (i == j)
					{
						continue;
					}

					const FVector2D& DiffPos = Positions[j] - Positions[i];
					float DistanceSq = DiffPos.SizeSquared();
					if (DistanceSq < SmoothLenSq)
					{
						float DiffLenSq = SmoothLenSq - DistanceSq;
						Densities[i] += DensityCoef * DiffLenSq * DiffLenSq * DiffLenSq;
					}
				}
			}
		}
	);
#endif
}

void ASPHSimulatorCPU::CalculatePressure()
{
#if 1
	for (int32 i = 0; i < NumParticles; ++i)
	{
		Pressures[i] = PressureStiffness * FMath::Max(FMath::Pow(Densities[i] / RestDensity, 7) - 1.0f, 0.0f);
	}
#else
	ParallelFor(
		NumParticles,
		[this](int32 i)
		{
			Pressures[i] = PressureStiffness * FMath::Max(FMath::Pow(Densities[i] / RestDensity, 7) - 1.0f, 0.0f);
		}
	);
#endif
}

void ASPHSimulatorCPU::ApplyPressure()
{
#if 1
	for (int32 i = 0; i < NumParticles; ++i)
	{
		FVector2D AccumPressure = FVector2D::ZeroVector;

		for (int32 j = 0; j < NumParticles; ++j)
		{
			if (i == j)
			{
				continue;
			}

			const FVector2D& DiffPos = Positions[j] - Positions[i];
			float DistanceSq = DiffPos.SizeSquared();
			float Distance = DiffPos.Size();
			if (DistanceSq < SmoothLenSq
				&& Densities[j] > SMALL_NUMBER && Distance > SMALL_NUMBER) // 0除算と、小さな値の除算ですごく大きな項になるのを回避
			{
				float DiffLen = SmoothLength - Distance;
#if 1
				// 数式と違うが、UnityGraphicsProgramming1がソースコードで使っていた式。こちらの方がなぜか安定するしフレームレートも上がる
				float AvgPressure = 0.5f * (Pressures[i] + Pressures[j]);
				AccumPressure += GradientPressureCoef * AvgPressure / Densities[j] * DiffLen * DiffLen / Distance * DiffPos;
#else
				float DiffPressure = Pressures[i] - Pressures[j];
				AccumPressure += GradientPressureCoef * DiffPressure / Densities[j] * DiffLen * DiffLen / Distance * DiffPos;
#endif
			}
		}

		if (Densities[i] > SMALL_NUMBER) // 0除算と、小さな値の除算ですごく大きな項になるのを回避
		{
			Accelerations[i] += AccumPressure / Densities[i];
		}
		else
		{
			(void)i;
		}
	}
#else
	ParallelFor(NumThreads,
		[this](int32 ThreadIndex)
		{
			for (int32 i = NumThreadParticles * ThreadIndex; i < NumThreadParticles * (ThreadIndex + 1) && i < NumParticles; ++i)
			{
				FVector2D AccumPressure = FVector2D::ZeroVector;

				for (int32 j = 0; j < NumParticles; ++j)
				{
					if (i == j)
					{
						continue;
					}

					const FVector2D& DiffPos = Positions[j] - Positions[i];
					float DistanceSq = DiffPos.SizeSquared();
					float Distance = DiffPos.Size();
					if (DistanceSq < SmoothLenSq
						&& Densities[j] > SMALL_NUMBER && Distance > SMALL_NUMBER) // 0除算と、小さな値の除算ですごく大きな項になるのを回避
					{
						float DiffLen = SmoothLength - Distance;
#if 1
						// 数式と違うが、UnityGraphicsProgramming1がソースコードで使っていた式。こちらの方がなぜか安定するしフレームレートも上がる
						float AvgPressure = 0.5f * (Pressures[i] + Pressures[j]);
						AccumPressure += GradientPressureCoef * AvgPressure / Densities[j] * DiffLen * DiffLen / Distance * DiffPos;
#else
						float DiffPressure = Pressures[i] - Pressures[j];
						AccumPressure += GradientPressureCoef * DiffPressure / Densities[j] * DiffLen * DiffLen / Distance * DiffPos;
#endif
					}
				}

				if (Densities[i] > SMALL_NUMBER) // 0除算と、小さな値の除算ですごく大きな項になるのを回避
				{
					Accelerations[i] += AccumPressure / Densities[i];
				}
				else
				{
					(void)i;
				}
			}
		}
	);
#endif
}

void ASPHSimulatorCPU::ApplyViscosity()
{
#if 1
	for (int32 i = 0; i < NumParticles; ++i)
	{
		FVector2D AccumViscosity = FVector2D::ZeroVector;

		for (int32 j = 0; j < NumParticles; ++j)
		{
			if (i == j)
			{
				continue;
			}

			const FVector2D& DiffPos = Positions[j] - Positions[i];
			float DistanceSq = DiffPos.SizeSquared();
			if (DistanceSq < SmoothLenSq
				&& Densities[j] > SMALL_NUMBER) // 0除算と、小さな値の除算ですごく大きな項になるのを回避
			{
				const FVector2D& DiffVel = Velocities[j] - Velocities[i];
				AccumViscosity += LaplacianViscosityCoef / Densities[j] * (SmoothLength - DiffPos.Size()) * DiffVel;
			}
		}

		if (Densities[i] > SMALL_NUMBER) // 0除算と、小さな値の除算ですごく大きな項になるのを回避
		{
			Accelerations[i] += Viscosity * AccumViscosity / Densities[i];
		}
		else
		{
			(void)i;
		}
	}
#else
	ParallelFor(NumThreads,
		[this](int32 ThreadIndex)
		{
			for (int32 i = NumThreadParticles * ThreadIndex; i < NumThreadParticles * (ThreadIndex + 1) && i < NumParticles; ++i)
			{
				FVector2D AccumViscosity = FVector2D::ZeroVector;

				for (int32 j = 0; j < NumParticles; ++j)
				{
					if (i == j)
					{
						continue;
					}

					const FVector2D& DiffPos = Positions[j] - Positions[i];
					float DistanceSq = DiffPos.SizeSquared();
					if (DistanceSq < SmoothLenSq
						&& Densities[j] > SMALL_NUMBER) // 0除算と、小さな値の除算ですごく大きな項になるのを回避
					{
						const FVector2D& DiffVel = Velocities[j] - Velocities[i];
						AccumViscosity += LaplacianViscosityCoef / Densities[j] * (SmoothLength - DiffPos.Size()) * DiffVel;
					}
				}

				if (Densities[i] > SMALL_NUMBER) // 0除算と、小さな値の除算ですごく大きな項になるのを回避
				{
					Accelerations[i] += Viscosity * AccumViscosity / Densities[i];
				}
				else
				{
					(void)i;
				}
			}
		}
	);
#endif
}

void ASPHSimulatorCPU::ApplyWallPenalty()
{
	//TODO: SPHって言っても加速度使わずにPBD使ってもいいはずなんだよな
#if 1
	for (int32 i = 0; i < NumParticles; ++i)
	{
		// 上境界
		Accelerations[i] += FMath::Max(0.0f, Positions[i].Y - WallBox.Max.Y) * WallStiffness * FVector2D(0.0f, -1.0f);
		// 下境界
		Accelerations[i] += FMath::Max(0.0f, WallBox.Min.Y - Positions[i].Y) * WallStiffness * FVector2D(0.0f, 1.0f);
		// 左境界
		Accelerations[i] += FMath::Max(0.0f, WallBox.Min.X - Positions[i].X) * WallStiffness * FVector2D(1.0f, 0.0f);
		// 右境界
		Accelerations[i] += FMath::Max(0.0f, Positions[i].X - WallBox.Max.X) * WallStiffness * FVector2D(-1.0f, 0.0f);
	}
#else
	ParallelFor(
		NumParticles,
		[this](int32 i)
		{
			// 上境界
			Accelerations[i] += FMath::Max(0.0f, Positions[i].Y - WallBox.Max.Y) * WallStiffness * FVector2D(0.0f, -1.0f);
			// 下境界
			Accelerations[i] += FMath::Max(0.0f, WallBox.Min.Y - Positions[i].Y) * WallStiffness * FVector2D(0.0f, 1.0f);
			// 左境界
			Accelerations[i] += FMath::Max(0.0f, WallBox.Min.X - Positions[i].X) * WallStiffness * FVector2D(1.0f, 0.0f);
			// 右境界
			Accelerations[i] += FMath::Max(0.0f, Positions[i].X - WallBox.Max.X) * WallStiffness * FVector2D(-1.0f, 0.0f);
		}
	);
#endif
}

void ASPHSimulatorCPU::Integrate(float DeltaSeconds)
{
#if 1
	for (int32 i = 0; i < NumParticles; ++i)
	{
		Accelerations[i] += FVector2D(0.0f, Gravity);
		Accelerations[i] += FVector2D(0.0f, Gravity);
		// 前進オイラー法
		Velocities[i] += Accelerations[i] * DeltaSeconds;
		Positions[i] += Velocities[i] * DeltaSeconds;
	}
#else
	ParallelFor(
		NumParticles,
		[this, DeltaSeconds](int32 i)
		{
			Accelerations[i] += FVector2D(0.0f, Gravity);
			Accelerations[i] += FVector2D(0.0f, Gravity);
			// 前進オイラー法
			Velocities[i] += Accelerations[i] * DeltaSeconds;
			Positions[i] += Velocities[i] * DeltaSeconds;
		}
	);
#endif
}

ASPHSimulatorCPU::ASPHSimulatorCPU()
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

void ASPHSimulatorCPU::PostRegisterAllComponents()
{
	Super::PostRegisterAllComponents();

	// Set Notification Delegate
	if (NiagaraComponent)
	{
		NiagaraComponent->OnSystemFinished.AddUniqueDynamic(this, &ASPHSimulatorCPU::OnNiagaraSystemFinished);
	}
}

void ASPHSimulatorCPU::SetDestroyOnSystemFinish(bool bShouldDestroyOnSystemFinish)
{
	bDestroyOnSystemFinish = bShouldDestroyOnSystemFinish ? 1 : 0;  
};

void ASPHSimulatorCPU::OnNiagaraSystemFinished(UNiagaraComponent* FinishedComponent)
{
	if (bDestroyOnSystemFinish)
	{
		SetLifeSpan(0.0001f);
	}
}

#if WITH_EDITOR
bool ASPHSimulatorCPU::GetReferencedContentObjects(TArray<UObject*>& Objects) const
{
	Super::GetReferencedContentObjects(Objects);

	if (UNiagaraSystem* System = NiagaraComponent->GetAsset())
	{
		Objects.Add(System);
	}

	return true;
}

void ASPHSimulatorCPU::ResetInLevel()
{
	if (NiagaraComponent)
	{
		NiagaraComponent->Activate(true);
		NiagaraComponent->ReregisterComponent();
	}
}
#endif // WITH_EDITOR

