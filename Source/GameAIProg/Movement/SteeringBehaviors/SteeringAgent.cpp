// Fill out your copyright notice in the Description page of Project Settings.

#include "SteeringAgent.h"


// Sets default values
ASteeringAgent::ASteeringAgent()
{
	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ASteeringAgent::BeginPlay()
{
	Super::BeginPlay();
}

void ASteeringAgent::BeginDestroy()
{
	Super::BeginDestroy();
}

// Called every frame
void ASteeringAgent::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (SteeringBehavior)
	{
		SteeringOutput Output = SteeringBehavior->CalculateSteering(DeltaTime, *this);

		// SteeringOutput.LinearVelocity passed to AddMovementInput; CharacterMovementComponent handles speed
		if (!Output.LinearVelocity.IsNearlyZero())
		{
			AddMovementInput(FVector{ Output.LinearVelocity, 0.f });
		}

		// SteeringOutput.AngularVelocity applied by incrementing Yaw; allows behaviors like Face
		// to control the agent's orientation independently of linear movement
		if (!FMath::IsNearlyZero(Output.AngularVelocity))
		{
			FRotator CurrentRotation = GetActorRotation();
			CurrentRotation.Yaw += Output.AngularVelocity;
			SetActorRotation(CurrentRotation);
		}
	}
}

// Called to bind functionality to input
void ASteeringAgent::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
}

void ASteeringAgent::SetSteeringBehavior(ISteeringBehavior* NewSteeringBehavior)
{
	SteeringBehavior = NewSteeringBehavior;
}