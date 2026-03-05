#include "Level_CombinedSteering.h"
#include "imgui.h"

ALevel_CombinedSteering::ALevel_CombinedSteering()
{
	PrimaryActorTick.bCanEverTick = true;
}

void ALevel_CombinedSteering::BeginPlay()
{
	Super::BeginPlay();

	// Wanderer: BlendedSteering of Seek (toward mouse target) and Wander
	WandererAgent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ -200.f, 0.f, 90.f }, FRotator::ZeroRotator);
	if (IsValid(WandererAgent))
	{
		pSeek = new Seek();
		pWander = new Wander();

		pSeek->SetTarget(MouseTarget);

		pBlendedSteering = new BlendedSteering({
			{ pSeek,   SeekWeight   },
			{ pWander, WanderWeight }
			});

		WandererAgent->SetIsAutoOrienting(true);
		WandererAgent->SetSteeringBehavior(pBlendedSteering);
		WandererAgent->SetDebugRenderingEnabled(CanDebugRender);
	}

	// Evader: PrioritySteering(EvadeWithRadius, Wander) — wanders freely, evades when wanderer enters EvadeRadius
	EvaderAgent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{ 200.f, 0.f, 90.f }, FRotator::ZeroRotator);
	if (IsValid(EvaderAgent))
	{
		pEvadeWithRadius = new EvadeWithRadius();
		pEvaderWander = new Wander();

		pPrioritySteering = new PrioritySteering({ pEvadeWithRadius, pEvaderWander });

		EvaderAgent->SetIsAutoOrienting(true);
		EvaderAgent->SetSteeringBehavior(pPrioritySteering);
		EvaderAgent->SetDebugRenderingEnabled(CanDebugRender);
	}
}

void ALevel_CombinedSteering::BeginDestroy()
{
	delete pBlendedSteering;  pBlendedSteering = nullptr;
	delete pSeek;             pSeek = nullptr;
	delete pWander;           pWander = nullptr;

	delete pPrioritySteering; pPrioritySteering = nullptr;
	delete pEvadeWithRadius;  pEvadeWithRadius = nullptr;
	delete pEvaderWander;     pEvaderWander = nullptr;

	Super::BeginDestroy();
}

void ALevel_CombinedSteering::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Update seek target to current mouse position each frame
	if (pSeek)
		pSeek->SetTarget(MouseTarget);

	// Update evade target to the wanderer's current world position and velocity each frame
	if (pEvadeWithRadius && IsValid(WandererAgent))
	{
		FTargetData WandererTarget;
		WandererTarget.Position = WandererAgent->GetPosition();
		WandererTarget.Orientation = WandererAgent->GetRotation();
		WandererTarget.LinearVelocity = WandererAgent->GetLinearVelocity();
		pEvadeWithRadius->SetTarget(WandererTarget);
	}

#pragma region UI
	{
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Game AI", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

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

		ImGui::Text("Combined Steering");
		ImGui::Spacing();

		if (ImGui::Checkbox("Debug Rendering", &CanDebugRender))
		{
			if (IsValid(WandererAgent)) WandererAgent->SetDebugRenderingEnabled(CanDebugRender);
			if (IsValid(EvaderAgent))   EvaderAgent->SetDebugRenderingEnabled(CanDebugRender);
		}

		ImGui::Checkbox("Trim World", &TrimWorld->bShouldTrimWorld);
		if (TrimWorld->bShouldTrimWorld)
		{
			ImGuiHelpers::ImGuiSliderFloatWithSetter("Trim Size",
				TrimWorld->GetTrimWorldSize(), 1000.f, 3000.f,
				[this](float InVal) { TrimWorld->SetTrimWorldSize(InVal); });
		}

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Wanderer - Blended Steering Weights");
		ImGui::Spacing();

		// Seek weight slider — controls how strongly the wanderer steers toward the mouse target
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek", SeekWeight, 0.f, 1.f,
			[this](float InVal)
			{
				SeekWeight = InVal;
				if (float* w = pBlendedSteering->GetWeight(pSeek))
					*w = SeekWeight;
			}, "%.2f");

		// Wander weight slider — controls how strongly random wander movement contributes to the blend
		ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander", WanderWeight, 0.f, 1.f,
			[this](float InVal)
			{
				WanderWeight = InVal;
				if (float* w = pBlendedSteering->GetWeight(pWander))
					*w = WanderWeight;
			}, "%.2f");

		ImGui::End();
	}
#pragma endregion
}