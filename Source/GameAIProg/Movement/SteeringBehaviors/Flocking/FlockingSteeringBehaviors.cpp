#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/SteeringHelpers.h"


// COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	// Seek toward the average position of all neighbors
	FVector2D const AvgPos = pFlock->GetAverageNeighborPos();
	Target.Position = AvgPos;
	return Seek::CalculateSteering(DeltaT, Agent);
}


// SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Output{};

	int const NrOfNeighbors = pFlock->GetNrOfNeighbors();
	if (NrOfNeighbors == 0) return Output;

	TArray<ASteeringAgent*> const& Neighbors = pFlock->GetNeighbors();
	FVector2D const AgentPos = Agent.GetPosition();

	for (int i = 0; i < NrOfNeighbors; ++i)
	{
		FVector2D const ToAgent = AgentPos - Neighbors[i]->GetPosition();
		float const Distance = ToAgent.Length();

		// Inversely proportional (y=1/x): closer neighbors exert stronger repulsion
		if (Distance > 0.f)
			Output.LinearVelocity += ToAgent.GetSafeNormal() * (1.f / Distance);
	}

	return Output;
}

// VELOCITY MATCH (FLOCKING)
SteeringOutput VelocityMatch::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Output{};

	//match the average neighbor velocity
	Output.LinearVelocity = pFlock->GetAverageNeighborVelocity();

	return Output;
}