/*=============================================================================*/
// SpacePartitioning.h: Contains Cell and CellSpace which are used to partition
// a space into segments. Cells contain pointers to all the agents within.
// These are used to avoid unnecessary distance comparisons to agents that are far away.

// Heavily based on chapter 3 of "Programming Game AI by Example" - Mat Buckland
/*=============================================================================*/

#pragma once

#include <vector>
#include <list>
#include "Movement/SteeringBehaviors/SteeringAgent.h"

// Axis-aligned bounding rectangle (min/max)
struct FRect
{
	float Left;   // Min X
	float Bottom; // Min Y
	float Right;  // Max X
	float Top;    // Max Y

	FRect(float Left = 0.f, float Bottom = 0.f, float Right = 0.f, float Top = 0.f)
		: Left(Left), Bottom(Bottom), Right(Right), Top(Top)
	{
	}
};

// A single cell in the spatial partition grid.
// Holds non-owning pointers to agents currently inside its bounds.
struct Cell final
{
	Cell(float Left, float Bottom, float Width, float Height)
		: BoundingBox(Left, Bottom, Left + Width, Bottom + Height)
	{
	}

	// Returns the four corners of this cell as world positions (for debug rendering)
	std::vector<FVector2D> GetRectPoints() const
	{
		return {
			FVector2D{ BoundingBox.Left,  BoundingBox.Bottom },
			FVector2D{ BoundingBox.Right, BoundingBox.Bottom },
			FVector2D{ BoundingBox.Right, BoundingBox.Top    },
			FVector2D{ BoundingBox.Left,  BoundingBox.Top    }
		};
	}

	std::list<ASteeringAgent*> Agents; // Non-owning; Flock remains owner
	FRect BoundingBox;
};

// Flat spatial partitioning grid.
class CellSpace final
{
public:
	CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities);

	// Adds an agent to the cell that contains its current position
	void AddAgent(ASteeringAgent& Agent);

	// Moves an agent from its old cell to its new cell if it has crossed a boundary
	void UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos);

	// Populates the Neighbors pool with agents within QueryRadius of Agent
	void RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius);

	const TArray<ASteeringAgent*>& GetNeighbors()    const { return Neighbors; }
	int                            GetNrOfNeighbors() const { return NrOfNeighbors; }

	// Clears all agent lists from every cell
	void EmptyCells();

	// Debug: draws grid lines and per-cell agent counts
	void RenderCells() const;

private:
	UWorld* pWorld{};

	std::vector<Cell> Cells;
	FVector2D CellOrigin{}; // Bottom-left corner of the entire grid

	float SpaceWidth;
	float SpaceHeight;
	int   NrOfRows;
	int   NrOfCols;
	float CellWidth;
	float CellHeight;

	TArray<ASteeringAgent*> Neighbors;
	int NrOfNeighbors{ 0 };

	// Converts a world position to a clamped (OutCol, OutRow) pair.
	// Shared by PositionToIndex and RegisterNeighbors so clamp logic lives in one place.
	void PositionToColRow(FVector2D const& Pos, int& OutCol, int& OutRow) const;

	// Returns the flat array index of the cell containing Pos
	int PositionToIndex(FVector2D const& Pos) const;

	// Returns true when two FRects overlap
	bool DoRectsOverlap(FRect const& RectA, FRect const& RectB) const;
};