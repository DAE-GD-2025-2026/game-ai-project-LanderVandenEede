#pragma once

#include <Movement/SteeringBehaviors/SteeringHelpers.h>
#include "Kismet/KismetMathLibrary.h"
#include "DrawDebugHelpers.h"

class ASteeringAgent;

// Base interface for all steering behaviors. Each derived class is an atomic behavior
// that takes character and world data as input and outputs a desired LinearVelocity and AngularVelocity.
class ISteeringBehavior
{
public:
	ISteeringBehavior() = default;
	virtual ~ISteeringBehavior() = default;

	// Returns the desired SteeringOutput for the given agent this frame
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) = 0;

	void SetTarget(const FTargetData& NewTarget) { Target = NewTarget; }

	template<class T, std::enable_if_t<std::is_base_of_v<ISteeringBehavior, T>>* = nullptr>
	T* As()
	{
		return static_cast<T*>(this);
	}

protected:
	FTargetData Target;
};

// SEEK
// Atomic behavior to steer the agent toward the target's position.
class Seek : public ISteeringBehavior
{
public:
	Seek() = default;
	virtual ~Seek() override = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

// FLEE
// Atomic behavior to steer the agent away from the target's position. Opposite of Seek.
class Flee : public ISteeringBehavior
{
public:
	Flee() = default;
	virtual ~Flee() override = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};


// ARRIVE
// Atomic behavior similar to Seek, but decelerates the agent within SlowRadius and stops within TargetRadius.
class Arrive : public ISteeringBehavior
{
public:
	Arrive() = default;
	virtual ~Arrive() override = default;

	void SetSlowRadius(float Radius) { SlowRadius = Radius; }
	void SetTargetRadius(float Radius) { TargetRadius = Radius; }

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

private:
	float SlowRadius{ 200.f };      // Agent begins decelerating upon entering this radius
	float TargetRadius{ 30.f };     // Agent's desired LinearVelocity drops to zero inside this radius
	float OriginalMaxSpeed{ -1.f }; // Agent's max linear speed on first call; used to compute decelerated speed
};

// FACE
// Atomic behavior to rotate the agent toward the target. Outputs AngularVelocity only; LinearVelocity is always zero.
class Face : public ISteeringBehavior
{
public:
	Face() = default;
	virtual ~Face() override = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

// PURSUIT
// Atomic behavior to steer the agent toward the target's predicted position. Similar to Seek.
class Pursuit : public Seek
{
public:
	Pursuit() = default;
	virtual ~Pursuit() override = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

// EVADE
// Atomic behavior to steer the agent away from the target's predicted position. Opposite of Pursuit.
class Evade : public Flee
{
public:
	Evade() = default;
	virtual ~Evade() override = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};


// WANDER
// Atomic behavior for randomised movement. Each frame a point on a wander circle projected in front
// of the agent is chosen as the seek target. WanderAngle is persisted and clamped by MaxAngleChange
// to keep movement smooth.
class Wander : public Seek
{
public:
	Wander() = default;
	virtual ~Wander() override = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

	void SetWanderOffset(float Offset) { OffsetDistance = Offset; }
	void SetWanderRadius(float Radius) { WanderRadius = Radius; }
	void SetMaxAngleChange(float Radians) { MaxAngleChange = Radians; }

protected:
	float OffsetDistance{ 150.f };                         // Distance along the agent's forward direction to the wander circle center
	float WanderRadius{ 80.f };                            // Radius of the wander circle
	float MaxAngleChange{ FMath::DegreesToRadians(45.f) }; // Max change applied to WanderAngle per frame
	float WanderAngle{ 0.f };                              // Last calculated wander angle; persisted between frames for smooth movement
};