// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "design/TemplateMethod.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/sensor_define.h"
#include <vector>
using std::vector;

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GlobalCostmap.generated.h"

namespace global_costmap
{
const int width = 101;
const int height = 101;
const float resolution = 2.5f;
const Pose origin = {};
} // namespace global_costmap

class GlobalCostmap : public TemplateMethod
{

	bool isDebug = true;
	int limmitIndex = 100;
	int crashIndex = 99;
	int warningIndex = 97;
	int minimumIndex = 1;
	float crashSize = 2.0f;
	float clearanceSize = 8.0f;
	OccupancyGrid globalCostmap;
	Geometry geometry;
	DetectedObjectArray detectedObjectArray;
	//===
	void InitializeGrid(OccupancyGrid &costmap);
	vector<Vector3> LimmitObjects2Range(OccupancyGrid &costmap,DetectedObjectArray &objectsWprld);
	void UpdateOccupancyGrid(OccupancyGrid &costmap,DetectedObjectArray &objectsWprld);
	void CalculateHeightMap(OccupancyGrid &costmap, int x, int y);
	//===
	void GetState() override;
	void SpinOnce() override;
	void SetState() override;
	void Start() override;

public:
	GlobalCostmap();
	~GlobalCostmap(){};
};

UCLASS()
class TESTPROJECT_API AGlobalCostmap : public AActor
{
	GENERATED_BODY()
	TemplateMethod *gc = new GlobalCostmap();
	bool bInitialized = false;
	void Start();
	void Update();

public:
	// Sets default values for this actor's properties
	AGlobalCostmap();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};
