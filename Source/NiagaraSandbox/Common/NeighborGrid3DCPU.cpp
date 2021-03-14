#include "NeighborGrid3DCPU.h"

void FNeighborGrid3DCPU::Initialize(int32 NumCellsX, int32 NumCellsY, int32 NumCellsZ, int32 MaxNeighborsPerCell, float CellSize, const FVector& WorldBBoxSize)
{
	check(NumCellsX > 0);
	check(NumCellsY > 0);
	check(NumCellsZ > 0);
	check(MaxNeighborsPerCell > 0);
	check(CellSize > KINDA_SMALL_NUMBER);
	check(WorldBBoxSize.X > KINDA_SMALL_NUMBER);
	check(WorldBBoxSize.Y > KINDA_SMALL_NUMBER);
	check(WorldBBoxSize.Z > KINDA_SMALL_NUMBER);

	_NumCellsX = NumCellsX;
	_NumCellsY = NumCellsY;
	_NumCellsZ = NumCellsZ;
	_MaxNeighborsPerCell = MaxNeighborsPerCell;

	_ParticleIndicesArray.SetNum(NumCellsX * NumCellsY * NumCellsZ * MaxNeighborsPerCell);

	_CellSize = CellSize;
	_WorldBBoxSize = WorldBBoxSize;
}

void FNeighborGrid3DCPU::Reset()
{
#if 0
	for (int32& ParticleIndex : _ParticleIndicesArray)
	{
		ParticleIndex = INDEX_NONE;
	}
#else
	// -1ÇÃèâä˙âªÇMemsetÇ≈çsÇ§
	FMemory::Memset(_ParticleIndicesArray.GetData(), 0xff, _ParticleIndicesArray.Num() * sizeof(_ParticleIndicesArray[0]));
#endif
}

void FNeighborGrid3DCPU::GetNumCells(int32& NumCellsX, int32& NumCellsY, int32& NumCellsZ) const
{
	NumCellsX = _NumCellsX;
	NumCellsY = _NumCellsY;
	NumCellsZ = _NumCellsZ;
}

int32 FNeighborGrid3DCPU::GetMaxNeighborsPerCell() const
{
	return _MaxNeighborsPerCell;
}

