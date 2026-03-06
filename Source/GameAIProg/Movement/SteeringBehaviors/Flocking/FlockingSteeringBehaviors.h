#pragma once
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"

class Flock;


// COHESION 
// Steers the agent toward the average position of its neighbors.
class Cohesion final : public Seek
{
public:
	Cohesion(Flock* const pFlock) : pFlock(pFlock) {};
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
private:
	Flock* pFlock = nullptr;
};


// SEPARATION 
// Steers the agent away from each neighbor, inversely proportional (y=1/x) to distance.
class Separation final : public ISteeringBehavior
{
public:
	Separation(Flock* const pFlock) : pFlock(pFlock) {};
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
private:
	Flock* pFlock = nullptr;
};


// VELOCITY MATCH
// Steers the agent to match the average LinearVelocity of its neighbors.
class VelocityMatch final : public ISteeringBehavior
{
public:
	VelocityMatch(Flock* const pFlock) : pFlock(pFlock) {};
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
private:
	Flock* pFlock = nullptr;
};