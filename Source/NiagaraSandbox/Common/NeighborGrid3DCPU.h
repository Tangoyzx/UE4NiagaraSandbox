#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"

struct FNeighborGrid3DCPU
{
private:
	TArray<int32> _ParticleIndicesArray;
	int32 _NumCellsX;
	int32 _NumCellsY;
	int32 _NumCellsZ;
	// The name is max neighbors but it is just the count of particle indices in the cell.
	// The word "Neighbor" represents "in the same cell" at NeighborGrid3D.
	int32 _MaxNeighborsPerCell;
	float _CellSize;
	FVector _WorldBBoxSize;

public:
	void Initialize(int32 NumCellsX, int32 NumCellsY, int32 NumCellsZ, int32 MaxNeighborsPerCell, float CellSize, const FVector& WorldBBoxSize);
	void Reset();
	void GetNumCells(int32& NumCellsX, int32& NumCellsY, int32& NumCellsZ) const;
	int32 GetMaxNeighborsPerCell() const;
};

