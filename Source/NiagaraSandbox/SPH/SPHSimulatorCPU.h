#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "GameFramework/Actor.h"
#include "SPHSimulatorCPU.generated.h"

UCLASS()
class ASPHSimulatorCPU : public AActor
{
	GENERATED_BODY()

public:
	ASPHSimulatorCPU();

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
	int32 NumParticles = 100;

	UPROPERTY(EditAnywhere)
	int32 NumIterations = 4;

	UPROPERTY(EditAnywhere)
	float FrameRate = 60.0f;

	UPROPERTY(EditAnywhere)
	FBox2D WallBox = FBox2D(FVector2D(0.0f, 0.0f), FVector2D(1600.0f, 900.0f));

	UPROPERTY(EditAnywhere)
	float WallStiffness = 3000.0f;

	UPROPERTY(EditAnywhere)
	float Gravity = -981.0f;

	UPROPERTY(EditAnywhere)
	float Mass = 80.0f;

	UPROPERTY(EditAnywhere)
	float SmoothLength = 50.0f;

	UPROPERTY(EditAnywhere)
	float RestDensity = 0.4f;

	UPROPERTY(EditAnywhere)
	float PressureStiffness = 5700.0f;

	UPROPERTY(EditAnywhere)
	float Viscosity = 30000.0f;

	UPROPERTY(EditAnywhere)
	float InitPosRadius = 400.0f;

private:
	void Simulate(float DeltaSeconds);
	void CalculateDensity();
	void CalculatePressure();
	void ApplyPressure();
	void ApplyViscocity();
	void ApplyWallPenalty();
	void Integrate(float DeltaSeconds);

private:
	TArray<FVector2D> Positions;
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

