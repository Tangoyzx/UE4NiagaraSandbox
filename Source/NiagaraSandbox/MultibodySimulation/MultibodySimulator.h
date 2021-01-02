#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MultibodySimulator.generated.h"

UCLASS()
class AMultibodySimulator : public AActor
{
	GENERATED_BODY()

protected:
	AMultibodySimulator();
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

protected:
	/** Constant of gravitation. The unit is m^3 * kg*^-1 * s^-2. */
	UPROPERTY(EditAnywhere, Category = Simulation, meta = (AllowPrivateAccess = "true"))
	float Gravity = 6.6743015E-11f;

private:
	UPROPERTY(Transient)
	TArray<class AActor*> MassPoints;
};

