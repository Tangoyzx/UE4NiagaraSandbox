#include "NeighborGrid3DCPU.h"

void FNeighborGrid3DCPU::Initialize(const FIntVector& NumCells, int32 MaxNeighborsPerCell)
{
	check(NumCells.X > 0);
	check(NumCells.Y > 0);
	check(NumCells.Z > 0);
	check(MaxNeighborsPerCell > 0);

	_NumCells = NumCells;
	_MaxNeighborsPerCell = MaxNeighborsPerCell;

	_ParticleIndicesArray.SetNum(NumCells.X * NumCells.Y * NumCells.Z * MaxNeighborsPerCell);
	_ParticleNeighborCountArray.SetNum(NumCells.X * NumCells.Y * NumCells.Z);
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
	// 0‚Ì‰Šú‰»‚ðMemset‚Ås‚¤
	FMemory::Memset(_ParticleNeighborCountArray.GetData(), 0, _ParticleNeighborCountArray.Num() * sizeof(_ParticleNeighborCountArray[0]));
	// -1‚Ì‰Šú‰»‚ðMemset‚Ås‚¤
	FMemory::Memset(_ParticleIndicesArray.GetData(), 0xff, _ParticleIndicesArray.Num() * sizeof(_ParticleIndicesArray[0]));
#endif
}

FIntVector FNeighborGrid3DCPU::GetNumCells() const
{
	return _NumCells;
}

bool FNeighborGrid3DCPU::IsValidCellIndex(const FIntVector& CellIndex) const
{
	return CellIndex.X >= 0 && CellIndex.X < _NumCells.X
		&& CellIndex.Y >= 0 && CellIndex.Y < _NumCells.Y
		&& CellIndex.Z >= 0 && CellIndex.Z < _NumCells.Z;
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
	// ³‚Ìfloat‚È‚çint32‚Ö‚Ì•ÏŠ·‚Í‚½‚¾‚ÌŠÛ‚ß‚É‚È‚é
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

int32 FNeighborGrid3DCPU::GetParticleNeighborCount(int32 LinearIndex) const
{
	return _ParticleNeighborCountArray[LinearIndex];
}

void FNeighborGrid3DCPU::SetParticleNeighborCount(int32 InLinearIndex, int32 InIncrement, int32& PreviousNeighborCount)
{
	PreviousNeighborCount = _ParticleNeighborCountArray[InLinearIndex];
	_ParticleNeighborCountArray[InLinearIndex] += InIncrement;
}

int32 FNeighborGrid3DCPU::GetParticleNeighbor(int32 NeighborGridLinearIndex) const
{
	return _ParticleIndicesArray[NeighborGridLinearIndex];
}

void FNeighborGrid3DCPU::SetParticleNeighbor(int32 NeighborGridLinearIndex, int32 ParticleIndex)
{
	_ParticleIndicesArray[NeighborGridLinearIndex] = ParticleIndex;
}
