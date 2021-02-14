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
	Positions3D.SetNum(NumParticles);

	Positions[0] = FVector2D::ZeroVector;
	Positions[1] = FVector2D(10.0f, 0.0f);
	Positions[2] = FVector2D(0.0f, 10.0f);

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

	ApplyBoundaryPenalty(DeltaSeconds);
	Integrate(DeltaSeconds);
}

void ASPHSimulatorCPU::ApplyBoundaryPenalty(float DeltaSeconds)
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

