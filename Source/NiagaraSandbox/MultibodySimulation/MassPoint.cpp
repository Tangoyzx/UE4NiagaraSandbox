#include "MassPoint.h"
#include "Components/StaticMeshComponent.h"

AMassPoint::AMassPoint()
{
	StaticMeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("StaticMeshComponent"));
	RootComponent = StaticMeshComponent;
}

void AMassPoint::BeginPlay()
{
	Super::BeginPlay();

	Velocity = InitialVelocity;
}

