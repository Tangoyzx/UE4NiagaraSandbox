#include "NeighborGrid3DCPU.h"

void FNeighborGrid3DCPU::Initialize(const FIntVector& NumCells, int32 MaxNeighborsPerCell, float CellSize, const FVector& WorldBBoxSize)
{
	check(NumCells.X > 0);
	check(NumCells.Y > 0);
	check(NumCells.Z > 0);
	check(MaxNeighborsPerCell > 0);
	check(CellSize > KINDA_SMALL_NUMBER);
	check(WorldBBoxSize.X > KINDA_SMALL_NUMBER);
	check(WorldBBoxSize.Y > KINDA_SMALL_NUMBER);
	check(WorldBBoxSize.Z > KINDA_SMALL_NUMBER);

	_NumCells = NumCells;
	_MaxNeighborsPerCell = MaxNeighborsPerCell;

	_ParticleIndicesArray.SetNum(NumCells.X * NumCells.Y * NumCells.Z * MaxNeighborsPerCell);
	_ParticleNeighborCountArray.SetNum(NumCells.X * NumCells.Y * NumCells.Z);

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
	for (int32& ParticleNeighborCount : _ParticleNeighborCountArray)
	{
		ParticleNeighborCount = 0;
	}
#else
	// 0ÇÃèâä˙âªÇMemsetÇ≈çsÇ§
	FMemory::Memset(_ParticleNeighborCountArray.GetData(), 0, _ParticleNeighborCountArray.Num() * sizeof(_ParticleNeighborCountArray[0]));
	// -1ÇÃèâä˙âªÇMemsetÇ≈çsÇ§
	FMemory::Memset(_ParticleIndicesArray.GetData(), 0xff, _ParticleIndicesArray.Num() * sizeof(_ParticleIndicesArray[0]));
#endif
}

FIntVector FNeighborGrid3DCPU::GetNumCells() const
{
	return _NumCells;
}

int32 FNeighborGrid3DCPU::MaxNeighborsPerCell() const
{
	return _MaxNeighborsPerCell;
}

FVector FNeighborGrid3DCPU::SimulationToUnit(const FVector& Simulation, const FTransform& SimulationToUnitTransform)
{
	return SimulationToUnitTransform.TransformPosition(Simulation);
}

FIntVector FNeighborGrid3DCPU::UnitToIndex(const FVector& Unit) const
{
	// ê≥ÇÃfloatÇ»ÇÁint32Ç÷ÇÃïœä∑ÇÕÇΩÇæÇÃä€ÇﬂÇ…Ç»ÇÈ
	return FIntVector(Unit * FVector(_NumCells));
}

int32 FNeighborGrid3DCPU::IndexToLinear(const FIntVector& Index) const
{
	return Index.X + Index.Y * _NumCells.X + Index.Z * _NumCells.X * _NumCells.Y;
}

int32 FNeighborGrid3DCPU::NeighborGridIndexToLinear(const FIntVector& Index, int32 NeighborIndex)
{
	return NeighborIndex + IndexToLinear(Index) * _MaxNeighborsPerCell;
}

void FNeighborGrid3DCPU::SetParticleNeighborCount(int32 InLinearIndex, int32 InIncrement, int32& PreviousNeighborCount)
{
	PreviousNeighborCount = _ParticleNeighborCountArray[InLinearIndex];
	_ParticleNeighborCountArray[InLinearIndex] += InIncrement;
}

