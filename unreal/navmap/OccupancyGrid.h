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
#include "OccupancyGrid.generated.h"

namespace local_costmap
{
const int width = 31;
const int height = 31;
const float resolution = 1.0f;
const Pose origin = {};
} // namespace local_costmap

//なるべく組み込みようにROSの定義は使わない
//Geometryをフル活用する場合は別
class OccupancyGridProcessor : public TemplateMethod
{
	bool isDebug = true;
	//definition for occupancy grid
	OccupancyGrid localCostmap;
	Rect gridRange={};
	int limmitIndex = 100;
	int crashIndex = 99;
	int warningIndex = 97;
	int minimumIndex = 1;
	float crashSize = 3.0f;
	float clearanceSize = 5.0f;

	//grid update
	void InitializeGrid(OccupancyGrid &costmap);
	vector<Vector3> LimmitObjects2Range(OccupancyGrid &costmap,DetectedObjectArray &objectsWorld);
	void UpdateOccupancyGrid(OccupancyGrid &costmap, vector<Vector3> &objects);
	void CalculateHeightMap(OccupancyGrid &costmap, int x, int y);
	void UpdateWorldGrid();
	DetectedObjectArray detectedObjectArray;
	vector<Vector3> objectsLocal;

	void GetState() override;
	void SpinOnce() override;
	void SetState() override;
	void Start() override;

public:
	OccupancyGridProcessor();
	~OccupancyGridProcessor();
};

UCLASS()
class TESTPROJECT_API AOccupancyGrid : public AActor
{
	GENERATED_BODY()
private:
	TemplateMethod *oc = new OccupancyGridProcessor();
	float offsetz = 1.2f;
	void Update();

public:
	AOccupancyGrid();
	~AOccupancyGrid();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};
