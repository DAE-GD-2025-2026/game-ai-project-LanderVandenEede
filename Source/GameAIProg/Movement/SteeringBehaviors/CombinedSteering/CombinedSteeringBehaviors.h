#pragma once
#include <vector>

#include "../Steering/SteeringBehaviors.h"


// BLENDED STEERING
// Combined behavior that computes a weighted average SteeringOutput across all WeightedBehaviors.
// Each behavior's output is scaled by its weight [0..1]; the accumulated result is divided by the total weight.
class BlendedSteering final : public ISteeringBehavior
{
public:
	struct WeightedBehavior
	{
		ISteeringBehavior* pBehavior = nullptr;
		float Weight = 0.f;

		WeightedBehavior(ISteeringBehavior* const pBehavior, float Weight) :
			pBehavior(pBehavior),
			Weight(Weight)
		{
		};
	};

	BlendedSteering(const std::vector<WeightedBehavior>& WeightedBehaviors);

	void AddBehaviour(const WeightedBehavior& WeightedBehavior) { WeightedBehaviors.push_back(WeightedBehavior); }
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

	float* GetWeight(ISteeringBehavior* const SteeringBehavior);

	// Returns a reference to the weighted behaviors; intended for weight adjustment only, not behavior modification
	std::vector<WeightedBehavior>& GetWeightedBehaviorsRef() { return WeightedBehaviors; }

private:
	std::vector<WeightedBehavior> WeightedBehaviors = {};

	// using ISteeringBehavior::SetTarget; // made private because targets need to be set on the individual behaviors, not the combined behavior
};


// PRIORITY STEERING
// Combined behavior that iterates an ordered list of behaviors and returns the first valid SteeringOutput.
// Uses SteeringOutput.IsValid; if no behavior returns valid, the last output is returned.
class PrioritySteering final : public ISteeringBehavior
{
public:
	PrioritySteering(const std::vector<ISteeringBehavior*>& priorityBehaviors)
		: m_PriorityBehaviors(priorityBehaviors)
	{
	}

	void AddBehaviour(ISteeringBehavior* const pBehavior) { m_PriorityBehaviors.push_back(pBehavior); }
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

private:
	std::vector<ISteeringBehavior*> m_PriorityBehaviors = {};

	// using ISteeringBehavior::SetTarget; // made private because targets need to be set on the individual behaviors, not the combined behavior
};


// EVADE WITH RADIUS
// Evade variant for use with PrioritySteering. Sets IsValid = false when the target is outside EvadeRadius,
// allowing PrioritySteering to fall through to the next behavior (e.g. Wander).
class EvadeWithRadius : public Evade
{
public:
	EvadeWithRadius() = default;
	virtual ~EvadeWithRadius() override = default;

	void SetEvadeRadius(float Radius) { EvadeRadius = Radius; }

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

private:
	float EvadeRadius{ 200.f }; // Target must be within this radius for the output to be valid
};