#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"

struct FNeighborGrid3DCPU
{
private:
	TArray<int32> _ParticleIndicesArray;
	TArray<int32> _ParticleNeighborCountArray;

	FIntVector _NumCells;
	// The name is max neighbors but it is just the count of particle indices in the cell.
	// The word "Neighbor" represents "in the same cell" at NeighborGrid3D.
	int32 _MaxNeighborsPerCell;
	float _CellSize;
	FVector _WorldBBoxSize;

public:
	void Initialize(const FIntVector& NumCells, int32 MaxNeighborsPerCell, float CellSize, const FVector& WorldBBoxSize);
	void Reset();
	FIntVector GetNumCells() const;
	// get MaxNeighborsPerCell
	int32 MaxNeighborsPerCell() const;
	static FVector SimulationToUnit(const FVector& Simulation, const FTransform& SimulationToUnitTransform);
	FIntVector UnitToIndex(const FVector& Unit) const;
	// cell index to linear index
	int32 IndexToLinear(const FIntVector& Index) const;
	// cell index to neighbor grid linear index
	int32 NeighborGridIndexToLinear(const FIntVector& Index, int32 NeighborIndex);
	// PreviousNeighborCount just become neigbor index (i.e. particle index) of the cell.
	void SetParticleNeighborCount(int32 InLinearIndex, int32 InIncrement, int32& PreviousNeighborCount);

};

