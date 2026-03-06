#pragma once

// Toggle this define to enable/disable spatial partitioning
// #define GAMEAI_USE_SPACE_PARTITIONING

#include "FlockingSteeringBehaviors.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/SteeringHelpers.h"
#include "Movement/SteeringBehaviors/CombinedSteering/CombinedSteeringBehaviors.h"
#include <memory>
#include "imgui.h"
#ifdef GAMEAI_USE_SPACE_PARTITIONING
#include "../SpacePartitioning/SpacePartitioning.h"
#endif

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

#ifdef GAMEAI_USE_SPACE_PARTITIONING
	// Space partitioning neighbor accessors (next week)
#else
	void RegisterNeighbors(ASteeringAgent* const Agent);
	int GetNrOfNeighbors() const { return NrOfNeighbors; }
	const TArray<ASteeringAgent*>& GetNeighbors() const { return Neighbors; }
#endif

	FVector2D GetAverageNeighborPos() const;
	FVector2D GetAverageNeighborVelocity() const;

	void SetTarget_Seek(FSteeringParams const& Target);

private:
	UWorld* pWorld{ nullptr };

	int FlockSize{ 0 };
	TArray<ASteeringAgent*> Agents{};

#ifdef GAMEAI_USE_SPACE_PARTITIONING
	// Space partitioning (next week)
#else
	// Memory pool: fixed-size array, NrOfNeighbors tracks how many slots are in use
	TArray<ASteeringAgent*> Neighbors{};
#endif

	float NeighborhoodRadius{ 200.f };
	int NrOfNeighbors{ 0 };

	ASteeringAgent* pAgentToEvade{ nullptr };

	// Flocking behaviors — owned by Flock
	Separation* pSeparationBehavior{ nullptr };
	Cohesion* pCohesionBehavior{ nullptr };
	VelocityMatch* pVelMatchBehavior{ nullptr };
	Seek* pSeekBehavior{ nullptr };
	Wander* pWanderBehavior{ nullptr };
	EvadeWithRadius* pEvadeBehavior{ nullptr };

	std::unique_ptr<BlendedSteering>  pBlendedSteering{};
	std::unique_ptr<PrioritySteering> pPrioritySteering{};

	// BlendedSteering weights
	float WeightSeparation{ 2.0f };
	float WeightCohesion{ 0.5f };
	float WeightAlignment{ 0.5f };
	float WeightSeek{ 0.2f };
	float WeightWander{ 0.4f };

	// UI and rendering
	bool DebugRenderSteering{ false };
	bool DebugRenderNeighborhood{ true };

	void RenderNeighborhood();
};