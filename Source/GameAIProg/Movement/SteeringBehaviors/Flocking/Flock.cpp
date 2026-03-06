#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Movement/SteeringBehaviors/SpacePartitioning/SpacePartitioning.h"
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
	OldPositions.SetNum(FlockSize);

	
	Neighbors.SetNum(FlockSize);

	// CellSpace: world is square centered on origin, size = WorldSize*2 each axis
	float const GridSize = WorldSize * 2.f;
	pCellSpace = std::make_unique<CellSpace>(pWorld, GridSize, GridSize, NrOfCellsX, NrOfCellsX, FlockSize);

	// Spawn all actors, auto-tick disabled so Flock controls update order
	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	for (int i = 0; i < FlockSize; ++i)
	{
		float const RandX = FMath::RandRange(-WorldSize, WorldSize);
		float const RandY = FMath::RandRange(-WorldSize, WorldSize);

		Agents[i] = pWorld->SpawnActor<ASteeringAgent>(AgentClass,
			FVector{ RandX, RandY, 90.f }, FRotator::ZeroRotator, SpawnParams);

		if (IsValid(Agents[i]))
		{
			Agents[i]->SetIsAutoOrienting(true);
			Agents[i]->SetActorTickEnabled(false); // Flock manually ticks each agent
			OldPositions[i] = Agents[i]->GetPosition();
			pCellSpace->AddAgent(*Agents[i]);
		}
	}

	// Create flocking behaviors, all take a Flock* for neighborhood access
	pSeparationBehavior = new Separation(this);
	pCohesionBehavior = new Cohesion(this);
	pVelMatchBehavior = new VelocityMatch(this);
	pSeekBehavior = new Seek();
	pWanderBehavior = new Wander();
	pEvadeBehavior = new EvadeWithRadius();

	// BlendedSteering: weighted average of all flocking behaviors
	pBlendedSteering = std::make_unique<BlendedSteering>(std::vector<BlendedSteering::WeightedBehavior>{
		{ pSeparationBehavior, WeightSeparation },
		{ pCohesionBehavior,   WeightCohesion },
		{ pVelMatchBehavior,   WeightAlignment },
		{ pSeekBehavior,       WeightSeek },
		{ pWanderBehavior,     WeightWander }
	});

	// PrioritySteering: evade first, flock otherwise
	pPrioritySteering = std::make_unique<PrioritySteering>(std::vector<ISteeringBehavior*>{
		pEvadeBehavior,
			pBlendedSteering.get()
	});

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
	// pCellSpace cleaned up by unique_ptr
}

void Flock::Tick(float DeltaTime)
{
	// Update evade target from the designated evade agent each tick
	if (IsValid(pAgentToEvade) && pEvadeBehavior)
	{
		FTargetData EvadeTarget;
		EvadeTarget.Position = pAgentToEvade->GetPosition();
		EvadeTarget.Orientation = pAgentToEvade->GetRotation();
		EvadeTarget.LinearVelocity = pAgentToEvade->GetLinearVelocity();
		pEvadeBehavior->SetTarget(EvadeTarget);
	}

	for (int i = 0; i < Agents.Num(); ++i)
	{
		ASteeringAgent* Agent = Agents[i];

		if (!IsValid(Agent)) continue;

		RegisterNeighbors(Agent);
		Agent->Tick(DeltaTime);

		if (bUseSpatialPartitioning)
			pCellSpace->UpdateAgentCell(*Agent, OldPositions[i]);
		OldPositions[i] = Agent->GetPosition();
	}
}

void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	if (bUseSpatialPartitioning)
	{
		pCellSpace->RegisterNeighbors(*pAgent, NeighborhoodRadius);
	}
	else
	{
		NrOfNeighbors = 0;
		FVector2D const AgentPos = pAgent->GetPosition();

		for (ASteeringAgent* Other : Agents)
		{
			if (Other == pAgent || !IsValid(Other)) continue;

			if (FVector2D::Distance(AgentPos, Other->GetPosition()) <= NeighborhoodRadius)
			{
				Neighbors[NrOfNeighbors] = Other;
				++NrOfNeighbors;
			}
		}
	}
}

int Flock::GetNrOfNeighbors() const
{
	return bUseSpatialPartitioning ? pCellSpace->GetNrOfNeighbors() : NrOfNeighbors;
}

const TArray<ASteeringAgent*>& Flock::GetNeighbors() const
{
	return bUseSpatialPartitioning ? pCellSpace->GetNeighbors() : Neighbors;
}

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D Avg = FVector2D::ZeroVector;
	int const N = GetNrOfNeighbors();
	if (N == 0) return Avg;

	const TArray<ASteeringAgent*>& Pool = GetNeighbors();
	for (int i = 0; i < N; ++i)
		Avg += Pool[i]->GetPosition();

	return Avg / static_cast<float>(N);
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D Avg = FVector2D::ZeroVector;
	int const N = GetNrOfNeighbors();
	if (N == 0) return Avg;

	const TArray<ASteeringAgent*>& Pool = GetNeighbors();
	for (int i = 0; i < N; ++i)
		Avg += Pool[i]->GetLinearVelocity();

	return Avg / static_cast<float>(N);
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	if (pSeekBehavior)
		pSeekBehavior->SetTarget(Target);
}

void Flock::RenderDebug()
{
	if (DebugRenderNeighborhood)
		RenderNeighborhood();

	if (bUseSpatialPartitioning && DebugRenderPartitions)
		pCellSpace->RenderCells();
}

void Flock::RenderNeighborhood()
{
	// Only renders for the first agent
	if (Agents.IsEmpty() || !IsValid(Agents[0])) return;

	ASteeringAgent* const FirstAgent = Agents[0];
	FVector const AgentWorldPos = FirstAgent->GetActorLocation();

	// Neighborhood radius circle, white
	DrawDebugCircle(pWorld, AgentWorldPos, NeighborhoodRadius, 32,
		FColor::White, false, -1.f, 0, 2.f, FVector(0, 1, 0), FVector(1, 0, 0));

	// Line to each neighbor, cyan
	int const N = GetNrOfNeighbors();
	const TArray<ASteeringAgent*>& Pool = GetNeighbors();
	for (int i = 0; i < N; ++i)
	{
		if (IsValid(Pool[i]))
			DrawDebugLine(pWorld, AgentWorldPos, Pool[i]->GetActorLocation(),
				FColor::Cyan, false, -1.f, 0, 1.f);
	}

	// Bounding box of neighborhood query
	if (bUseSpatialPartitioning)
	{
		FVector const BoxExtent{ NeighborhoodRadius, NeighborhoodRadius, 1.f };
		DrawDebugBox(pWorld, AgentWorldPos, BoxExtent, FColor::Green, false, -1.f, 0, 2.f);
	}
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
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

		// Debug toggles
		ImGui::Checkbox("Debug Neighborhood", &DebugRenderNeighborhood);
		ImGui::Checkbox("Debug Steering", &DebugRenderSteering);
		for (ASteeringAgent* Agent : Agents)
			if (IsValid(Agent)) Agent->SetDebugRenderingEnabled(DebugRenderSteering);

		// Spatial partitioning toggle, switches neighbor source at runtime
		bool bSPToggle = bUseSpatialPartitioning;
		if (ImGui::Checkbox("Spatial Partitioning", &bSPToggle) && bSPToggle != bUseSpatialPartitioning)
		{
			bUseSpatialPartitioning = bSPToggle;
			if (bUseSpatialPartitioning)
			{
				// Rebuild cell registration from scratch when switching on
				pCellSpace->EmptyCells();
				for (ASteeringAgent* Agent : Agents)
					if (IsValid(Agent)) pCellSpace->AddAgent(*Agent);
			}
		}

		if (bUseSpatialPartitioning)
			ImGui::Checkbox("Debug Partitions", &DebugRenderPartitions);

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Neighborhood R", NeighborhoodRadius, 10.f, 500.f,
			[this](float v) { NeighborhoodRadius = v; });

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Separation", WeightSeparation, 0.f, 5.f,
			[this](float v) { WeightSeparation = v; if (float* w = pBlendedSteering->GetWeight(pSeparationBehavior)) *w = v; }, "%.2f");

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Cohesion", WeightCohesion, 0.f, 5.f,
			[this](float v) { WeightCohesion = v; if (float* w = pBlendedSteering->GetWeight(pCohesionBehavior)) *w = v; }, "%.2f");

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Alignment", WeightAlignment, 0.f, 5.f,
			[this](float v) { WeightAlignment = v; if (float* w = pBlendedSteering->GetWeight(pVelMatchBehavior)) *w = v; }, "%.2f");

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek", WeightSeek, 0.f, 5.f,
			[this](float v) { WeightSeek = v; if (float* w = pBlendedSteering->GetWeight(pSeekBehavior)) *w = v; }, "%.2f");

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander", WeightWander, 0.f, 5.f,
			[this](float v) { WeightWander = v; if (float* w = pBlendedSteering->GetWeight(pWanderBehavior)) *w = v; }, "%.2f");

		ImGui::End();
	}
#endif
}