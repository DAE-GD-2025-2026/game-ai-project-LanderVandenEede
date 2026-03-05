#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//***********
// SEEK
//***********
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Output{};

	// Desired LinearVelocity: normalised direction from agent to target
	FVector2D const ToTarget = Target.Position - Agent.GetPosition();
	Output.LinearVelocity = ToTarget.GetSafeNormal();

	// Debug: arrow in the desired LinearVelocity direction
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugDirectionalArrow(
			Agent.GetWorld(),
			Agent.GetActorLocation(),
			Agent.GetActorLocation() + FVector{ Output.LinearVelocity * 80.f, 0.f },
			15.f, FColor::Green, false, -1.f, 0, 2.f
		);
	}

	return Output;
}

//***********
// FLEE
//***********
SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Output{};

	// Desired LinearVelocity: normalised direction away from target
	FVector2D const AwayFromTarget = Agent.GetPosition() - Target.Position;
	Output.LinearVelocity = AwayFromTarget.GetSafeNormal();

	// Debug: arrow in the desired LinearVelocity direction
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugDirectionalArrow(
			Agent.GetWorld(),
			Agent.GetActorLocation(),
			Agent.GetActorLocation() + FVector{ Output.LinearVelocity * 80.f, 0.f },
			15.f, FColor::Red, false, -1.f, 0, 2.f
		);
	}

	return Output;
}

//***********
// ARRIVE
//***********
SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Output{};

	// Captured once on the first call; used to restore full speed outside SlowRadius
	if (OriginalMaxSpeed < 0.f)
		OriginalMaxSpeed = Agent.GetMaxLinearSpeed();

	FVector2D const ToTarget = Target.Position - Agent.GetPosition();
	float const Distance = ToTarget.Length();

	// Inside TargetRadius: agent has arrived, desired LinearVelocity is zero
	if (Distance <= TargetRadius)
	{
		Agent.SetMaxLinearSpeed(0.f);
		Output.LinearVelocity = FVector2D::ZeroVector;
		return Output;
	}

	// Between TargetRadius and SlowRadius: max linear speed scales linearly with remaining distance
	if (Distance <= SlowRadius)
	{
		float const SpeedFraction = (Distance - TargetRadius) / (SlowRadius - TargetRadius);
		Agent.SetMaxLinearSpeed(OriginalMaxSpeed * SpeedFraction);
	}
	else
	{
		// Outside SlowRadius: full max linear speed
		Agent.SetMaxLinearSpeed(OriginalMaxSpeed);
	}

	Output.LinearVelocity = ToTarget.GetSafeNormal();

	// Debug: SlowRadius (blue), TargetRadius (orange), arrow in desired LinearVelocity direction
	if (Agent.GetDebugRenderingEnabled())
	{
		FVector const AgentLoc = Agent.GetActorLocation();
		DrawDebugCircle(Agent.GetWorld(), AgentLoc, SlowRadius, 32, FColor::Blue, false, -1.f, 0, 2.f, FVector(0, 1, 0), FVector(1, 0, 0));
		DrawDebugCircle(Agent.GetWorld(), AgentLoc, TargetRadius, 32, FColor::Orange, false, -1.f, 0, 2.f, FVector(0, 1, 0), FVector(1, 0, 0));
		DrawDebugDirectionalArrow(
			Agent.GetWorld(), AgentLoc,
			AgentLoc + FVector{ Output.LinearVelocity * 80.f, 0.f },
			15.f, FColor::Cyan, false, -1.f, 0, 2.f
		);
	}

	return Output;
}

//***********
// FACE
//***********
SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Output{};
	Output.LinearVelocity = FVector2D::ZeroVector; // AngularVelocity only; no linear movement

	// Desired orientation from direction between agent and target positions
	FVector2D const ToTarget = Target.Position - Agent.GetPosition();
	if (ToTarget.IsNearlyZero())
		return Output;

	float const DesiredYaw = FMath::RadiansToDegrees(FMath::Atan2(ToTarget.Y, ToTarget.X));
	float       DeltaYaw = DesiredYaw - Agent.GetRotation();

	// Unwound to [-180, 180] to rotate via the shortest arc
	DeltaYaw = FMath::UnwindDegrees(DeltaYaw);

	// AngularVelocity clamped to max angular speed for this frame
	float const MaxAngSpeed = Agent.GetMaxAngularSpeed();
	Output.AngularVelocity = FMath::Clamp(DeltaYaw, -MaxAngSpeed * DeltaT, MaxAngSpeed * DeltaT);

	// Debug: arrow toward target showing desired facing direction
	if (Agent.GetDebugRenderingEnabled())
	{
		FVector const AgentLoc = Agent.GetActorLocation();
		DrawDebugDirectionalArrow(
			Agent.GetWorld(), AgentLoc,
			AgentLoc + FVector{ ToTarget.GetSafeNormal() * 80.f, 0.f },
			15.f, FColor::Yellow, false, -1.f, 0, 2.f
		);
	}

	return Output;
}

//***********
// PURSUIT
//***********
SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	float const Distance = FVector2D::Distance(Target.Position, Agent.GetPosition());
	float const AgentSpeed = Agent.GetMaxLinearSpeed();

	// Predicted time to reach target: t = d / v
	float const PredictTime = (AgentSpeed > KINDA_SMALL_NUMBER) ? Distance / AgentSpeed : 0.f;

	// Predicted position: target extrapolated along its LinearVelocity by PredictTime
	FVector2D const PredictedPos = Target.Position + Target.LinearVelocity * PredictTime;

	// Debug: sphere at predicted position, arrow from agent toward it
	if (Agent.GetDebugRenderingEnabled())
	{
		FVector const AgentLoc = Agent.GetActorLocation();
		FVector const PredictLoc{ PredictedPos, AgentLoc.Z };
		DrawDebugSphere(Agent.GetWorld(), PredictLoc, 12.f, 8, FColor::Magenta, false, -1.f, 0, 2.f);
		DrawDebugDirectionalArrow(
			Agent.GetWorld(), AgentLoc, PredictLoc,
			15.f, FColor::Magenta, false, -1.f, 0, 2.f
		);
	}

	// Seek toward the predicted position
	Target.Position = PredictedPos;
	return Seek::CalculateSteering(DeltaT, Agent);
}

//***********
// EVADE
//***********
SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	float const Distance = FVector2D::Distance(Target.Position, Agent.GetPosition());
	float const AgentSpeed = Agent.GetMaxLinearSpeed();

	// Predicted position calculated identically to Pursuit: t = d / v, extrapolated along target's LinearVelocity
	float const PredictTime = (AgentSpeed > KINDA_SMALL_NUMBER) ? Distance / AgentSpeed : 0.f;
	FVector2D const PredictedPos = Target.Position + Target.LinearVelocity * PredictTime;

	// Debug: sphere at predicted position, arrow in desired LinearVelocity direction
	if (Agent.GetDebugRenderingEnabled())
	{
		FVector const AgentLoc = Agent.GetActorLocation();
		FVector const PredictLoc{ PredictedPos, AgentLoc.Z };
		DrawDebugSphere(Agent.GetWorld(), PredictLoc, 12.f, 8, FColor::Orange, false, -1.f, 0, 2.f);
		DrawDebugDirectionalArrow(
			Agent.GetWorld(), AgentLoc,
			AgentLoc + FVector{ (Agent.GetPosition() - PredictedPos).GetSafeNormal() * 80.f, 0.f },
			15.f, FColor::Orange, false, -1.f, 0, 2.f
		);
	}

	// Flee from the predicted position
	Target.Position = PredictedPos;
	return Flee::CalculateSteering(DeltaT, Agent);
}

//***********
// WANDER
//***********
SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	// Random offset clamped to MaxAngleChange added to WanderAngle each frame for smooth movement
	float const AngleOffset = FMath::RandRange(-MaxAngleChange, MaxAngleChange);
	WanderAngle += AngleOffset;

	// Wander circle center: agent position offset along forward direction by OffsetDistance
	FVector2D const AgentForward = FVector2D{ FMath::Cos(FMath::DegreesToRadians(Agent.GetRotation())),
											  FMath::Sin(FMath::DegreesToRadians(Agent.GetRotation())) };
	FVector2D const CircleCenter = Agent.GetPosition() + AgentForward * OffsetDistance;

	// Point on the wander circle at the current WanderAngle becomes the seek target
	FVector2D const WanderPoint{
		CircleCenter.X + FMath::Cos(WanderAngle) * WanderRadius,
		CircleCenter.Y + FMath::Sin(WanderAngle) * WanderRadius
	};

	// Seek toward the wander point to produce the desired LinearVelocity
	Target.Position = WanderPoint;
	SteeringOutput Output = Seek::CalculateSteering(DeltaT, Agent);

	// Debug: wander circle at offset position, sphere and line at the current wander point
	if (Agent.GetDebugRenderingEnabled())
	{
		FVector const AgentLoc = Agent.GetActorLocation();
		FVector const CircleLoc{ CircleCenter, AgentLoc.Z };
		FVector const WanderLoc{ WanderPoint, AgentLoc.Z };

		DrawDebugCircle(Agent.GetWorld(), CircleLoc, WanderRadius, 32,
			FColor::Blue, false, -1.f, 0, 2.f, FVector(0, 1, 0), FVector(1, 0, 0));
		DrawDebugSphere(Agent.GetWorld(), WanderLoc, 8.f, 8, FColor::White, false, -1.f, 0, 2.f);
		DrawDebugLine(Agent.GetWorld(), AgentLoc, WanderLoc, FColor::White, false, -1.f, 0, 2.f);
	}

	return Output;
}