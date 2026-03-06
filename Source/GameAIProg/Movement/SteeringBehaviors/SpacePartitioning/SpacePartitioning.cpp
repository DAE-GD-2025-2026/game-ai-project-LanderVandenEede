#include "SpacePartitioning.h"
#include "DrawDebugHelpers.h"

CellSpace::CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities)
	: pWorld{ pWorld }
	, SpaceWidth{ Width }
	, SpaceHeight{ Height }
	, NrOfRows{ Rows }
	, NrOfCols{ Cols }
	, CellWidth{ Width / static_cast<float>(Cols) }
	, CellHeight{ Height / static_cast<float>(Rows) }
{
	// Grid is centered on the world origin
	CellOrigin = FVector2D{ -Width * 0.5f, -Height * 0.5f };

	// Build the flat cell array row by row, bottom to top
	Cells.reserve(Rows * Cols);
	for (int Row = 0; Row < Rows; ++Row)
	{
		for (int Col = 0; Col < Cols; ++Col)
		{
			float const Left = CellOrigin.X + Col * CellWidth;
			float const Bottom = CellOrigin.Y + Row * CellHeight;
			Cells.emplace_back(Left, Bottom, CellWidth, CellHeight);
		}
	}

	Neighbors.SetNum(MaxEntities);
}

void CellSpace::PositionToColRow(FVector2D const& Pos, int& OutCol, int& OutRow) const
{
	// Convert world position to grid coordinates, clamped to valid range
	OutCol = FMath::Clamp(static_cast<int>((Pos.X - CellOrigin.X) / CellWidth), 0, NrOfCols - 1);
	OutRow = FMath::Clamp(static_cast<int>((Pos.Y - CellOrigin.Y) / CellHeight), 0, NrOfRows - 1);
}

int CellSpace::PositionToIndex(FVector2D const& Pos) const
{
	int Col, Row;
	PositionToColRow(Pos, Col, Row);
	return Row * NrOfCols + Col;
}

bool CellSpace::DoRectsOverlap(FRect const& RectA, FRect const& RectB) const
{
	return RectA.Left <= RectB.Right &&
		RectA.Right >= RectB.Left &&
		RectA.Bottom <= RectB.Top &&
		RectA.Top >= RectB.Bottom;
}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	int const Index = PositionToIndex(Agent.GetPosition());
	Cells[Index].Agents.push_back(&Agent);
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	int const OldIndex = PositionToIndex(OldPos);
	int const NewIndex = PositionToIndex(Agent.GetPosition());

	// Only update if the agent has actually crossed a cell boundary
	if (OldIndex != NewIndex)
	{
		Cells[OldIndex].Agents.remove(&Agent);
		Cells[NewIndex].Agents.push_back(&Agent);
	}
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	NrOfNeighbors = 0;

	FVector2D const AgentPos = Agent.GetPosition();

	float const RadiusSq = QueryRadius * QueryRadius;

	int ColMin, RowMin, ColMax, RowMax;
	PositionToColRow(FVector2D{ AgentPos.X - QueryRadius, AgentPos.Y - QueryRadius }, ColMin, RowMin);
	PositionToColRow(FVector2D{ AgentPos.X + QueryRadius, AgentPos.Y + QueryRadius }, ColMax, RowMax);

	for (int Row = RowMin; Row <= RowMax; ++Row)
	{
		for (int Col = ColMin; Col <= ColMax; ++Col)
		{
			for (ASteeringAgent* pOther : Cells[Row * NrOfCols + Col].Agents)
			{
				if (pOther == &Agent) continue;

				FVector2D const Delta = AgentPos - pOther->GetPosition();
				if (Delta.SizeSquared() <= RadiusSq)
				{
					Neighbors[NrOfNeighbors++] = pOther;
				}
			}
		}
	}
}

void CellSpace::EmptyCells()
{
	for (Cell& cell : Cells)
		cell.Agents.clear();
}

void CellSpace::RenderCells() const
{
	float const Z = 90.f; // Draw at agent height

	for (Cell const& cell : Cells)
	{
		FRect const& bb = cell.BoundingBox;

		// Cell boundary in grey
		FVector const BL{ bb.Left,  bb.Bottom, Z };
		FVector const BR{ bb.Right, bb.Bottom, Z };
		FVector const TR{ bb.Right, bb.Top,    Z };
		FVector const TL{ bb.Left,  bb.Top,    Z };

		DrawDebugLine(pWorld, BL, BR, FColor::Silver, false, -1.f, 0, 1.f);
		DrawDebugLine(pWorld, BR, TR, FColor::Silver, false, -1.f, 0, 1.f);
		DrawDebugLine(pWorld, TR, TL, FColor::Silver, false, -1.f, 0, 1.f);
		DrawDebugLine(pWorld, TL, BL, FColor::Silver, false, -1.f, 0, 1.f);

		// Agent count at cell center
		int const Count = static_cast<int>(cell.Agents.size());
		if (Count > 0)
		{
			FVector const Center{
				(bb.Left + bb.Right) * 0.5f,
				(bb.Bottom + bb.Top) * 0.5f,
				Z + 10.f
			};
			DrawDebugString(pWorld, Center, FString::FromInt(Count), nullptr, FColor::Yellow, 0.f, false, 1.f);
		}
	}
}