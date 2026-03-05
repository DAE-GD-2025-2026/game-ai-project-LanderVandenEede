#include "CombinedSteeringBehaviors.h"
#include <algorithm>
#include <numeric>
#include "../SteeringAgent.h"


// BLENDED STEERING
BlendedSteering::BlendedSteering(const std::vector<WeightedBehavior>& WeightedBehaviors)
	: WeightedBehaviors(WeightedBehaviors)
{
};

SteeringOutput BlendedSteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput BlendedOutput{};
	float TotalWeight = 0.f;

	// Accumulate each behavior's output scaled by its weight
	for (WeightedBehavior const& WB : WeightedBehaviors)
	{
		if (WB.pBehavior && WB.Weight > 0.f)
		{
			SteeringOutput const BehaviorOutput = WB.pBehavior->CalculateSteering(DeltaT, Agent);
			BlendedOutput.LinearVelocity += BehaviorOutput.LinearVelocity * WB.Weight;
			BlendedOutput.AngularVelocity += BehaviorOutput.AngularVelocity * WB.Weight;
			TotalWeight += WB.Weight;
		}
	}

	// Divide by total weight to produce the weighted average
	if (TotalWeight > 0.f)
	{
		BlendedOutput.LinearVelocity /= TotalWeight;
		BlendedOutput.AngularVelocity /= TotalWeight;
	}

	// Debug: arrow showing the final blended desired LinearVelocity
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugDirectionalArrow(
			Agent.GetWorld(),
			Agent.GetActorLocation(),
			Agent.GetActorLocation() + FVector{ BlendedOutput.LinearVelocity * 80.f, 0.f },
			30.f, FColor::Red, false, -1.f, 0, 3.f
		);
	}

	return BlendedOutput;
}

float* BlendedSteering::GetWeight(ISteeringBehavior* const SteeringBehavior)
{
	auto it = find_if(WeightedBehaviors.begin(),
		WeightedBehaviors.end(),
		[SteeringBehavior](const WeightedBehavior& Elem)
		{
			return Elem.pBehavior == SteeringBehavior;
		}
	);

	if (it != WeightedBehaviors.end())
		return &it->Weight;

	return nullptr;
}

// PRIORITY STEERING
SteeringOutput PrioritySteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	// Returns the first valid SteeringOutput from the ordered priority list
	for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
	{
		Steering = pBehavior->CalculateSteering(DeltaT, Agent);

		if (Steering.IsValid)
			break;
	}

	// If no behavior produced a valid output, the last result is returned
	return Steering;
}


// EVADE WITH RADIUS
SteeringOutput EvadeWithRadius::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	float const Distance = FVector2D::Distance(Target.Position, Agent.GetPosition());

	// Target is outside EvadeRadius: output is invalid so PrioritySteering falls through to the next behavior
	if (Distance > EvadeRadius)
	{
		SteeringOutput Invalid{};
		Invalid.IsValid = false;
		return Invalid;
	}

	// Debug: evade radius circle showing the activation boundary
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugCircle(Agent.GetWorld(), Agent.GetActorLocation(), EvadeRadius, 32,
			FColor::Orange, false, -1.f, 0, 2.f, FVector(0, 1, 0), FVector(1, 0, 0));
	}

	return Evade::CalculateSteering(DeltaT, Agent);
}