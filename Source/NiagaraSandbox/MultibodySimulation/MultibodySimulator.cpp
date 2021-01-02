#include "MultibodySimulator.h"
#include "MassPoint.h"
#include "Kismet/GameplayStatics.h"

AMultibodySimulator::AMultibodySimulator()
{
	PrimaryActorTick.bCanEverTick = true;
}

void AMultibodySimulator::BeginPlay()
{
	Super::BeginPlay();
	UGameplayStatics::GetAllActorsOfClass(this, AMassPoint::StaticClass(), MassPoints);
}

void AMultibodySimulator::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	for (int i = 0; i < MassPoints.Num(); ++i)
	{
		AMassPoint* MassPoint = Cast<AMassPoint>(MassPoints[i]);

		for (int j = 0; j < MassPoints.Num(); ++j)
		{
			if (i == j)
			{
				continue;
			}

			AMassPoint* AnotherMassPoint = Cast<AMassPoint>(MassPoints[j]);

			// �O�t���[���̈ʒu��p���đ��x���v�Z����
			const FVector& Diff = AnotherMassPoint->GetActorLocation() - MassPoint->GetActorLocation();
			float DistSquared = Diff.SizeSquared();
			const FVector& Acceleration
				= Gravity * AnotherMassPoint->Mass
				/ FMath::Max(DistSquared, 1.0f) // 0���Z�ɂȂ�Ȃ��悤�A�Œ዗����1m�Ƃ���
				* Diff.GetSafeNormal();

			MassPoint->Velocity = MassPoint->Velocity + Acceleration * DeltaSeconds;
		}
	}

	for (int i = 0; i < MassPoints.Num(); ++i)
	{
		AMassPoint* MassPoint = Cast<AMassPoint>(MassPoints[i]);
		MassPoint->SetActorLocation(MassPoint->GetActorLocation() + MassPoint->Velocity * DeltaSeconds);
	}
}

