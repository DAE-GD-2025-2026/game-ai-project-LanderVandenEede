#pragma once

#include "FlockingSteeringBehaviors.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/SteeringHelpers.h"
#include "Movement/SteeringBehaviors/CombinedSteering/CombinedSteeringBehaviors.h"
#include <memory>
#include "imgui.h"


class CellSpace;

class Flock final
{
public:
	Flock(
		UWorld* pWorld,
		TSubclassOf<ASteeringAgent> AgentClass,
		int FlockSize = 10,
		float WorldSize = 100.f,
		ASteeringAgent* const pAgentToEvade = nullptr,
		bool bTrimWorld = false);

	~Flock();

	void Tick(float DeltaTime);
	void RenderDebug();
	void ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize);

	// Neighbor accessors, return from CellSpace or flat pool depending on bUseSpatialPartitioning
	void RegisterNeighbors(ASteeringAgent* const Agent);
	int GetNrOfNeighbors() const;
	const TArray<ASteeringAgent*>& GetNeighbors() const;

	FVector2D GetAverageNeighborPos() const;
	FVector2D GetAverageNeighborVelocity() const;

	void SetTarget_Seek(FSteeringParams const& Target);

private:
	UWorld* pWorld{ nullptr };

	int FlockSize{ 0 };
	TArray<ASteeringAgent*> Agents{};

	// Old positions tracked per agent for UpdateAgentCell
	TArray<FVector2D> OldPositions{};

	// Flat neighbor memory pool (used when spatial partitioning is off)
	TArray<ASteeringAgent*> Neighbors{};
	int NrOfNeighbors{ 0 };

	float NeighborhoodRadius{ 200.f };

	ASteeringAgent* pAgentToEvade{ nullptr };

	// Flocking behaviors, owned by Flock
	Separation* pSeparationBehavior{ nullptr };
	Cohesion* pCohesionBehavior{ nullptr };
	VelocityMatch* pVelMatchBehavior{ nullptr };
	Seek* pSeekBehavior{ nullptr };
	Wander* pWanderBehavior{ nullptr };
	EvadeWithRadius* pEvadeBehavior{ nullptr };

	std::unique_ptr<BlendedSteering>  pBlendedSteering{};
	std::unique_ptr<PrioritySteering> pPrioritySteering{};

	// Spatial partitioning, forward declared, allocated in Flock.cpp
	std::unique_ptr<CellSpace> pCellSpace{};
	bool bUseSpatialPartitioning{ false };
	int NrOfCellsX{ 20 };

	// BlendedSteering weights
	float WeightSeparation{ 2.0f };
	float WeightCohesion{ 0.5f };
	float WeightAlignment{ 0.5f };
	float WeightSeek{ 0.2f };
	float WeightWander{ 0.4f };

	// UI and rendering
	bool DebugRenderSteering{ false };
	bool DebugRenderNeighborhood{ true };
	bool DebugRenderPartitions{ true };

	void RenderNeighborhood();
};