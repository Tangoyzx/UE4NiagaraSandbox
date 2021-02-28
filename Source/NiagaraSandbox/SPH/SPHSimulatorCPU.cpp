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

	Positions[0] = FVector2D(4.0f, 2.0f);
	Positions[1] = FVector2D(8.0f, 5.0f);
	Positions[2] = FVector2D(12.0f, 3.0f);

	Velocities[0] = FVector2D::ZeroVector;
	Velocities[1] = FVector2D::ZeroVector;
	Velocities[2] = FVector2D::ZeroVector;

	for (int32 i = 0; i < NumParticles; ++i)
	{
		Positions3D[i] = FVector(0.0f, Positions[i].X, Positions[i].Y);
	}

	// Tick()で設定しても、レベルにNiagaraSystemが最初から配置されていると、初回のスポーンでは配列は初期値を使ってしまい
	//間に合わないのでBeginPlay()でも設定する
	NiagaraComponent->SetNiagaraVariableInt("NumParticles", NumParticles);
	SetNiagaraArrayVector(NiagaraComponent, FName("Positions"), Positions3D);

	DensityCoef = Mass * 4.0f / PI / FMath::Pow(SmoothLength, 8);
	GradientPressureCoef = Mass * -30.0f / PI / FMath::Pow(SmoothLength, 5);
	LaplacianViscosityCoef = Mass * 20.0f / 3.0f / PI / FMath::Pow(SmoothLength, 5);
}

void ASPHSimulatorCPU::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (DeltaSeconds > KINDA_SMALL_NUMBER)
	{
		// DeltaSecondsの値の変動に関わらず、NumIterationで均等分割する。フレームレートの変化を考慮していない方法。
		float SubStepDeltaSeconds = DeltaSeconds / NumIterations;

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
	// TODO:初期化はあとで加速度を計算するようになったら不要になるのであとで消す
	for (int32 i = 0; i < NumParticles; ++i)
	{
		Accelerations[i] = FVector2D::ZeroVector;
	}

	CalculateDensity();
	CalculatePressure();
	ApplyPressure();
	ApplyViscocity();
	ApplyBoundaryPenalty();
	Integrate(DeltaSeconds);
}

void ASPHSimulatorCPU::CalculateDensity()
{
	static float SmoothLenSq = SmoothLength * SmoothLength;

	for (int32 i = 0; i < NumParticles; ++i)
	{
		Densities[i] = 0.0f;
	}

	for (int32 i = 0; i < NumParticles; ++i)
	{
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

void ASPHSimulatorCPU::CalculatePressure()
{
	static float SmoothLenSq = SmoothLength * SmoothLength; 
	for (int32 i = 0; i < NumParticles; ++i)
	{
		Pressures[i] = PressureStiffness * FMath::Max(FMath::Pow(Densities[i] / RestDensity, 7), 0.0f);
	}
}

void ASPHSimulatorCPU::ApplyPressure()
{
	// TODO:メンバ変数にした方がいいかも。他の関数でも計算している
	static float SmoothLenSq = SmoothLength * SmoothLength;

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
			if (DistanceSq < SmoothLenSq)
			{
				float AvgPressure = 0.5f * (Pressures[i] + Pressures[j]);
				float DiffLen = SmoothLength - DiffPos.Size();

				float Distance = DiffPos.Size();
				if (Densities[j] > SMALL_NUMBER && Distance > SMALL_NUMBER) // 0除算回避
				{
					AccumPressure += GradientPressureCoef * AvgPressure / Densities[j] * DiffLen * DiffLen / Distance * DiffPos;
				}
			}
		}

		if (Densities[i] > SMALL_NUMBER) // 0除算回避
		{
			Accelerations[i] += AccumPressure / Densities[i];
		}
	}
}

void ASPHSimulatorCPU::ApplyViscocity()
{
	// TODO:メンバ変数にした方がいいかも。他の関数でも計算している
	static float SmoothLenSq = SmoothLength * SmoothLength;

	for (int32 i = 0; i < NumParticles; ++i)
	{
		FVector2D AccumViscocity = FVector2D::ZeroVector;

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
				const FVector2D& DiffVel = Velocities[j] - Velocities[i];
				if (Densities[j] > SMALL_NUMBER) // 0除算回避
				{
					AccumViscocity += LaplacianViscosityCoef / Densities[j] * (SmoothLength - DiffPos.Size()) * DiffVel;
				}
			}
		}

		if (Densities[i] > SMALL_NUMBER) // 0除算回避
		{
			Accelerations[i] += Viscosity * AccumViscocity / Densities[i];
		}
	}
}

void ASPHSimulatorCPU::ApplyBoundaryPenalty()
{
	//TODO: SPHって言っても加速度使わずにPBD使ってもいいはずなんだよな
	for (int32 i = 0; i < NumParticles; ++i)
	{
		// 上境界
		Accelerations[i] += FMath::Max(0.0f, Positions[i].Y - BoundaryBox.Max.Y) * BoundaryStiffness * FVector2D(0.0f, -1.0f);
		// 下境界
		Accelerations[i] += FMath::Max(0.0f, BoundaryBox.Min.Y - Positions[i].Y) * BoundaryStiffness * FVector2D(0.0f, 1.0f);
		// 左境界
		Accelerations[i] += FMath::Max(0.0f, BoundaryBox.Min.X - Positions[i].X) * BoundaryStiffness * FVector2D(1.0f, 0.0f);
		// 右境界
		Accelerations[i] += FMath::Max(0.0f, Positions[i].X - BoundaryBox.Max.X) * BoundaryStiffness * FVector2D(-1.0f, 0.0f);
	}
}

void ASPHSimulatorCPU::Integrate(float DeltaSeconds)
{
	for (int32 i = 0; i < NumParticles; ++i)
	{
		Accelerations[i] += FVector2D(0.0f, Gravity);
	}

	// 前進オイラー法
	for (int32 i = 0; i < NumParticles; ++i)
	{
		Velocities[i] += Accelerations[i] * DeltaSeconds;
	}

	for (int32 i = 0; i < NumParticles; ++i)
	{
		Positions[i] += Velocities[i] * DeltaSeconds;
	}
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

