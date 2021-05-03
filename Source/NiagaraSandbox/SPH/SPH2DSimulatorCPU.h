#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "GameFramework/Actor.h"
#include "../Common/NeighborGrid3DCPU.h"
#include "SPH2DSimulatorCPU.generated.h"

UCLASS(MinimalAPI)
// ANiagaraActorを参考にしている
class ASPH2DSimulatorCPU : public AActor
{
	GENERATED_BODY()

public:
	ASPH2DSimulatorCPU();

	virtual void PostRegisterAllComponents() override;

	virtual void BeginPlay() override;
	virtual void Tick( float DeltaSeconds ) override;

	/** Set true for this actor to self-destruct when the Niagara system finishes, false otherwise */
	UFUNCTION(BlueprintCallable)
	void SetDestroyOnSystemFinish(bool bShouldDestroyOnSystemFinish);

private:
	/** Pointer to System component */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	class UNiagaraComponent* NiagaraComponent;

#if WITH_EDITORONLY_DATA
	// Reference to sprite visualization component
	UPROPERTY()
	class UBillboardComponent* SpriteComponent;

	// Reference to arrow visualization component
	UPROPERTY()
	class UArrowComponent* ArrowComponent;

#endif

	/** True for this actor to self-destruct when the Niagara system finishes, false otherwise */
	UPROPERTY()
	uint32 bDestroyOnSystemFinish : 1;

	/** Callback when Niagara system finishes. */
	UFUNCTION(CallInEditor)
	void OnNiagaraSystemFinished(UNiagaraComponent* FinishedComponent);

	UPROPERTY(EditAnywhere)
	int32 NumThreads = 4;

	UPROPERTY(EditAnywhere)
	bool bUseNeighborGrid3D = true;

	UPROPERTY(EditAnywhere)
	int32 NumParticles = 1000;

	UPROPERTY(EditAnywhere)
	int32 NumIterations = 4;

	UPROPERTY(EditAnywhere)
	float FrameRate = 60.0f;

	UPROPERTY(EditAnywhere)
	FBox2D WallBox = FBox2D(FVector2D(-4.5f, -4.5f), FVector2D(4.5f, 4.5f));

	UPROPERTY(EditAnywhere)
	float WallStiffness = 3000.0f;

	UPROPERTY(EditAnywhere)
	float Gravity = -9.81f;

	UPROPERTY(EditAnywhere)
	float Mass = 0.08f;

	UPROPERTY(EditAnywhere)
	float SmoothLength = 0.5f;

	UPROPERTY(EditAnywhere)
	float RestDensity = 4.0f;

	UPROPERTY(EditAnywhere)
	float PressureStiffness = 0.57f;

	UPROPERTY(EditAnywhere)
	float Viscosity = 3.0f;

	UPROPERTY(EditAnywhere)
	float InitPosRadius = 4.0f;

	UPROPERTY(EditAnywhere)
	int32 NumCellsX = 10;

	UPROPERTY(EditAnywhere)
	int32 NumCellsY = 10;

	UPROPERTY(EditAnywhere)
	int32 MaxNeighborsPerCell = 8;

	UPROPERTY(EditAnywhere)
	FVector2D WorldBBoxSize = FVector2D(10.0f, 10.0f);

private:
	void Simulate(float DeltaSeconds);
	void CalculateDensity(int32 ParticleIdx, int32 AnotherParticleIdx);
	void CalculatePressure(int32 ParticleIdx);
	void ApplyPressure(int32 ParticleIdx, int32 AnotherParticleIdx);
	void ApplyViscosity(int32 ParticleIdx, int32 AnotherParticleIdx);
	void ApplyWallPenalty(int32 ParticleIdx);
	void Integrate(int32 ParticleIdx, float DeltaSeconds);

private:
	TArray<FVector2D> Positions;
	TArray<FLinearColor> Colors;
	TArray<FVector2D> Velocities;
	// 加速度は毎フレーム計算するのでフレーム間のひきつぎはないのだが、使用メモリやTArrayの生成負荷をおさえるために
	// 使いまわしている
	TArray<FVector2D> Accelerations;
	TArray<float> Densities;
	TArray<float> Pressures;
	TArray<FVector> Positions3D;
	float DensityCoef = 0.0f;
	float GradientPressureCoef = 0.0f;
	float LaplacianViscosityCoef = 0.0f;
	float SmoothLenSq = 0.0f;
	int32 NumThreadParticles = 0.0f;
	FNeighborGrid3DCPU NeighborGrid3D;
	FTransform LocalToUnitTransform;

public:
	/** Returns NiagaraComponent subobject **/
	class UNiagaraComponent* GetNiagaraComponent() const { return NiagaraComponent; }
#if WITH_EDITORONLY_DATA
	/** Returns SpriteComponent subobject **/
	class UBillboardComponent* GetSpriteComponent() const { return SpriteComponent; }
	/** Returns ArrowComponent subobject **/
	class UArrowComponent* GetArrowComponent() const { return ArrowComponent; }
#endif

#if WITH_EDITOR
	// AActor interface
	virtual bool GetReferencedContentObjects(TArray<UObject*>& Objects) const override;
	// End of AActor interface

	/** Reset this actor in the level.*/
	void ResetInLevel();
#endif // WITH_EDITOR
};

