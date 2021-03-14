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

public:
	void Initialize(const FIntVector& NumCells, int32 MaxNeighborsPerCell);
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
	int32 GetParticleNeighborCount(int32 LinearIndex) const;
	// PreviousNeighborCount just become neigbor index (i.e. particle index) of the cell.
	// As if incrementation cause over MaxNeighborsPerCell, _ParticleNeighborCountArray[InLinearIndex] is incremented.
	void SetParticleNeighborCount(int32 InLinearIndex, int32 InIncrement, int32& PreviousNeighborCount);
	int32 GetParticleNeighbor(int32 NeighborGridLinearIndex) const;
	void SetParticleNeighbor(int32 NeighborGridLinearIndex, int32 ParticleIndex);
};

