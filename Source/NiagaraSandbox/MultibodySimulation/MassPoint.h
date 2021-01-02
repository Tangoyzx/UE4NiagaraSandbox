#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MassPoint.generated.h"

UCLASS()
class AMassPoint : public AActor
{
	GENERATED_BODY()

public:
	AMassPoint();

	/** Static mesh component. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Simulation, meta = (AllowPrivateAccess = "true"))
	class UStaticMeshComponent* StaticMeshComponent = nullptr;

	/** Mass. The unit is kilo gram. */
	UPROPERTY(EditAnywhere, Category = Simulation, meta = (AllowPrivateAccess = "true"))
	float Mass = 1.0f;

	/** Initial velocity. */
	UPROPERTY(EditAnywhere, Category = Simulation, meta = (AllowPrivateAccess = "true"))
	FVector InitialVelocity = FVector::ZeroVector;

	FVector Velocity = FVector::ZeroVector;

protected:
	virtual void BeginPlay() override;
};

