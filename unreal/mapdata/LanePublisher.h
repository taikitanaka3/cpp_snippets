// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#define _USE_MATH_DEFINES
#include "design/TemplateMethod.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/waypoint_define.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "LanePublisher.generated.h"

class LanePublisher : public TemplateMethod
{
private:
	float offsetz = 1.1f;
	//Waypoint MakeStraight();
	Pose MakeStraightWaypoints(vector<Waypoint> &waypoints, Pose initialPose, float d, float len);
	Lane MakeRightLane();

	//===
	void GetState() override;
	void SpinOnce() override;
	void SetState() override;
	void Start() override;
	//===
public:
	LanePublisher();
	~LanePublisher();
};

UCLASS()
class TESTPROJECT_API ALanePublisher : public AActor
{
	GENERATED_BODY()
	TemplateMethod *way = new LanePublisher();
	bool bInitialized = false;
	void Update();


public:
	// Sets default values for this actor's properties
	ALanePublisher();
	~ALanePublisher();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};
