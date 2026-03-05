// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include <memory>
#include "CombinedSteeringBehaviors.h"
#include "GameAIProg/Shared/Level_Base.h"
#include "GameAIProg/Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "Level_CombinedSteering.generated.h"

UCLASS()
class GAMEAIPROG_API ALevel_CombinedSteering : public ALevel_Base
{
	GENERATED_BODY()

public:
	ALevel_CombinedSteering();
	virtual void Tick(float DeltaTime) override;

protected:
	virtual void BeginPlay() override;
	virtual void BeginDestroy() override;

private:
	bool CanDebugRender = false;

	// Wanderer agent — uses BlendedSteering(Seek + Wander) to roam toward the mouse target
	ASteeringAgent* WandererAgent{ nullptr };
	Seek* pSeek{ nullptr };
	Wander* pWander{ nullptr };
	BlendedSteering* pBlendedSteering{ nullptr };

	// Evader agent — uses PrioritySteering(EvadeWithRadius, Wander); evades the wanderer when within EvadeRadius
	ASteeringAgent* EvaderAgent{ nullptr };
	EvadeWithRadius* pEvadeWithRadius{ nullptr };
	Wander* pEvaderWander{ nullptr };
	PrioritySteering* pPrioritySteering{ nullptr };

	// BlendedSteering weights, exposed to ImGui sliders
	float SeekWeight{ 0.5f };
	float WanderWeight{ 0.5f };
};