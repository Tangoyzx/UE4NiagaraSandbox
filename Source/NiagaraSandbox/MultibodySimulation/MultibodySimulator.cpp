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

	if (DeltaSeconds < KINDA_SMALL_NUMBER)
	{
		return;
	}

	float Delta = DeltaSeconds / NumIteration;

	for (int32 IterCount = 0; IterCount < NumIteration; ++IterCount)
	{
		for (int32 i = 0; i < MassPoints.Num(); ++i)
		{
			AMassPoint* MassPoint = Cast<AMassPoint>(MassPoints[i]);

			for (int j = 0; j < MassPoints.Num(); ++j)
			{
				if (i == j)
				{
					continue;
				}

				AMassPoint* AnotherMassPoint = Cast<AMassPoint>(MassPoints[j]);

				// 前フレームの位置を用いて速度を計算する
				const FVector& Diff = AnotherMassPoint->Position - MassPoint->Position;
				float DistSquared = Diff.SizeSquared();
				const FVector& Acceleration
					= Gravity * AnotherMassPoint->Mass
					/ FMath::Max(DistSquared, 1.0f) // 0除算にならないよう、最低距離を1mとする
					* Diff.GetSafeNormal();

				MassPoint->Velocity += Acceleration * Delta;
			}
		}

		for (int32 i = 0; i < MassPoints.Num(); ++i)
		{
			AMassPoint* MassPoint = Cast<AMassPoint>(MassPoints[i]);
			MassPoint->Position += MassPoint->Velocity * Delta;
		}
	}

	for (int32 i = 0; i < MassPoints.Num(); ++i)
	{
		AMassPoint* MassPoint = Cast<AMassPoint>(MassPoints[i]);
		MassPoint->SetActorLocation(MassPoint->Position);
	}
}

