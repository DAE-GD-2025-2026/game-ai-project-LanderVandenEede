#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"

Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{ pWorld }
	, FlockSize{ FlockSize }
	, pAgentToEvade{ pAgentToEvade }
{
	Agents.SetNum(FlockSize);

	//max possible neighbors
	Neighbors.SetNum(FlockSize);

	// Spawn all steeringActors at random positions within the world
	for (int i = 0; i < FlockSize; ++i)
	{
		float const RandX = FMath::RandRange(-WorldSize, WorldSize);
		float const RandY = FMath::RandRange(-WorldSize, WorldSize);
		Agents[i] = pWorld->SpawnActor<ASteeringAgent>(AgentClass, FVector{ RandX, RandY, 90.f }, FRotator::ZeroRotator);
		if (IsValid(Agents[i]))
		Agents[i]->SetIsAutoOrienting(true);
	}

	// Create flocking behavior
	pSeparationBehavior = new Separation(this);
	pCohesionBehavior = new Cohesion(this);
	pVelMatchBehavior = new VelocityMatch(this);
	pSeekBehavior = new Seek();
	pWanderBehavior = new Wander();
	pEvadeBehavior = new EvadeWithRadius();

	//weighted average of all flocking behaviors
	pBlendedSteering = std::make_unique<BlendedSteering>(std::vector<BlendedSteering::WeightedBehavior>{
		{ pSeparationBehavior, WeightSeparation },
		{ pCohesionBehavior,   WeightCohesion },
		{ pVelMatchBehavior,   WeightAlignment },
		{ pSeekBehavior,       WeightSeek },
		{ pWanderBehavior,     WeightWander }
	});

	//evade the target agent when within radius, otherwise flock
	pPrioritySteering = std::make_unique<PrioritySteering>(std::vector<ISteeringBehavior*>{
		pEvadeBehavior,
			pBlendedSteering.get()
	});

	// Assign PrioritySteering to every steeringAgent
	for (ASteeringAgent* Agent : Agents)
	{
		if (IsValid(Agent))
			Agent->SetSteeringBehavior(pPrioritySteering.get());
	}
}

Flock::~Flock()
{
	delete pSeparationBehavior;
	delete pCohesionBehavior;
	delete pVelMatchBehavior;
	delete pSeekBehavior;
	delete pWanderBehavior;
	delete pEvadeBehavior;
}

void Flock::Tick(float DeltaTime)
{
	// Update evade target from the evade agent's current state each tick
	if (IsValid(pAgentToEvade) && pEvadeBehavior)
	{
		FTargetData EvadeTarget;
		EvadeTarget.Position = pAgentToEvade->GetPosition();
		EvadeTarget.Orientation = pAgentToEvade->GetRotation();
		EvadeTarget.LinearVelocity = pAgentToEvade->GetLinearVelocity();
		pEvadeBehavior->SetTarget(EvadeTarget);
	}

	for (ASteeringAgent* Agent : Agents)
	{
		if (!IsValid(Agent)) continue;

		RegisterNeighbors(Agent);
		Agent->Tick(DeltaTime);
	}
}

void Flock::RenderDebug()
{
	if (DebugRenderNeighborhood)
		RenderNeighborhood();
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	{
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Game AI", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("WASD: move cam");
		ImGui::Text("Scrollwheel: zoom cam");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

		// Debug rendering toggles
		ImGui::Checkbox("Debug Neighborhood", &DebugRenderNeighborhood);
		ImGui::Checkbox("Debug Steering", &DebugRenderSteering);

		for (ASteeringAgent* Agent : Agents)
			if (IsValid(Agent)) Agent->SetDebugRenderingEnabled(DebugRenderSteering);

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		// Neighborhood radius slider
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Neighborhood R", NeighborhoodRadius, 10.f, 500.f,
			[this](float v) { NeighborhoodRadius = v; });

		// Per-behavior weight sliders — update the BlendedSteering's weight list directly
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Separation", WeightSeparation, 0.f, 5.f,
			[this](float v)
			{
				WeightSeparation = v;
				if (float* w = pBlendedSteering->GetWeight(pSeparationBehavior)) *w = v;
			}, "%.2f");

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Cohesion", WeightCohesion, 0.f, 5.f,
			[this](float v)
			{
				WeightCohesion = v;
				if (float* w = pBlendedSteering->GetWeight(pCohesionBehavior)) *w = v;
			}, "%.2f");

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Alignment", WeightAlignment, 0.f, 5.f,
			[this](float v)
			{
				WeightAlignment = v;
				if (float* w = pBlendedSteering->GetWeight(pVelMatchBehavior)) *w = v;
			}, "%.2f");

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek", WeightSeek, 0.f, 5.f,
			[this](float v)
			{
				WeightSeek = v;
				if (float* w = pBlendedSteering->GetWeight(pSeekBehavior)) *w = v;
			}, "%.2f");

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander", WeightWander, 0.f, 5.f,
			[this](float v)
			{
				WeightWander = v;
				if (float* w = pBlendedSteering->GetWeight(pWanderBehavior)) *w = v;
			}, "%.2f");

		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
	// Debug render: neighborhood radius circle and neighbor lines for the first agent only
	if (Agents.IsEmpty() || !IsValid(Agents[0])) return;

	ASteeringAgent* const FirstAgent = Agents[0];
	FVector const AgentWorldPos = FirstAgent->GetActorLocation();

	// Neighborhood radius circle
	DrawDebugCircle(pWorld, AgentWorldPos, NeighborhoodRadius, 32,
		FColor::White, false, -1.f, 0, 2.f, FVector(0, 1, 0), FVector(1, 0, 0));

	// Line to each neighbor
	for (int i = 0; i < NrOfNeighbors; ++i)
	{
		if (IsValid(Neighbors[i]))
		{
			DrawDebugLine(pWorld, AgentWorldPos, Neighbors[i]->GetActorLocation(),
				FColor::Cyan, false, -1.f, 0, 1.f);
		}
	}
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	NrOfNeighbors = 0;

	FVector2D const AgentPos = pAgent->GetPosition();

	for (ASteeringAgent* OtherAgent : Agents)
	{
		// Exclude the agent from its own neighborhood
		if (OtherAgent == pAgent || !IsValid(OtherAgent)) continue;

		float const Distance = FVector2D::Distance(AgentPos, OtherAgent->GetPosition());
		if (Distance <= NeighborhoodRadius)
		{
			// Write directly into the pre-allocated slot — memory pool pattern
			Neighbors[NrOfNeighbors] = OtherAgent;
			++NrOfNeighbors;
		}
	}
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D AvgPosition = FVector2D::ZeroVector;
	if (NrOfNeighbors == 0) return AvgPosition;

	for (int i = 0; i < NrOfNeighbors; ++i)
		AvgPosition += Neighbors[i]->GetPosition();

	return AvgPosition / static_cast<float>(NrOfNeighbors);
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D AvgVelocity = FVector2D::ZeroVector;
	if (NrOfNeighbors == 0) return AvgVelocity;

	for (int i = 0; i < NrOfNeighbors; ++i)
		AvgVelocity += Neighbors[i]->GetLinearVelocity();

	return AvgVelocity / static_cast<float>(NrOfNeighbors);
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	// Forward the mouse target to the Seek behavior inside BlendedSteering
	if (pSeekBehavior)
		pSeekBehavior->SetTarget(Target);
}