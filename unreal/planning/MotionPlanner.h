// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "design/TemplateMethod.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include <map>
#include <vector>
using std::map;
using std::vector;
//Unreal Defined
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MotionPlanner.generated.h"

class MotionPlanner : public TemplateMethod
{
private:
	bool isDebug = true;
	//for the local navgation
	Geometry geometry;
	VehicleState vehicleState;
	Path localPath = {};
	Path globalPath = {};
	GetPlan getPlan;
	Vector3 goal;
	float overrideScale = 0.1f;
	int laneIndex;
	int newLaneIndex;

	//===global
	int OverRide(int currentLane);
	void SearchNearestWaypoints(float x, float y);
	void LookAheadModel();
	//===
	void GetState() override;
	void SpinOnce() override;
	void SetState() override;
	void Start() override;
	//===
public:
	//==========
	MotionPlanner();
	~MotionPlanner();
};

UCLASS()
class TESTPROJECT_API AMotionPlanner : public AActor
{
	GENERATED_BODY()
	//LinearFeedback lf = {};
	TemplateMethod *mp = new MotionPlanner();
	bool bInitialized = false;
	void DrawPoints(const vector<Vector3> &position, FColor color, float size);
	bool isDebug = true;
	void Update();

public:
	// Sets default values for this actor's properties
	AMotionPlanner();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};
