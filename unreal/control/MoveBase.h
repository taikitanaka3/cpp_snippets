// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "design/TemplateMethod.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/waypoint_define.h"
#include "define/ctrl_define.h"
#include "design/StrategyPattern.h"
#include "define/ackermann_define.h"

#include "EmergencyStop.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MoveBase.generated.h"

class MoveBase : public TemplateMethod
{
private:

	//ControlStrategy ctr(new PurePursuit());
	//ctr.Control(1);
	//const変数
	bool isDebug = true;
	float dt;
	//Variables
	Geometry geometry;
	VehicleState vehicleState;
	Vector3 pos2Goal;
	Vector3 goal;
	AckermannDrive ackermannDrive;
	SteerCmd steerCmd;
	AccelCmd accelCmd;
	ControlCommand controlCommand;

	float deltaThrottleOld = 0.f;
	float deltaSteeringOld = 0.f;

	//=steer
	void PurePursuitControl();
	void LinearControl();
	float SteerControl();
	float ThrottleControl(float current);

	//===
	void GetState() override;
	void SpinOnce() override;
	void SetState() override;
	void Start() override;

	//===
public:
	//==========

	MoveBase();
	~MoveBase();
};

UCLASS()
class TESTPROJECT_API AMoveBase : public AActor
{
	GENERATED_BODY()
	bool isDebug = true;
	TemplateMethod *eStop = new EmergencyStop();
	TemplateMethod *mv = new MoveBase();
public:
	// Sets default values for this actor's properties
	AMoveBase();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};
