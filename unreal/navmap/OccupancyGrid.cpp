// Fill out your copyright notice in the Description page of Project Settings.
#include "OccupancyGrid.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/math_define.h"
#include "msgs/ros_msgs.h"
#include "msgs/auto_msgs.h"
#include "msgs/sensor_msgs.h"
#include "Costmap.h"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <map>
using std::cout;
using std::endl;
//====
#include "define/unreal_define.h"
#include "DrawDebugHelpers.h"
#include "Math/Color.h"
//Local OccupancyGridProcessor

OccupancyGridProcessor::OccupancyGridProcessor()
{
	cout << "create local grid" << endl;
}

OccupancyGridProcessor::~OccupancyGridProcessor()
{
	cout << "occ destructor called";
}
void OccupancyGridProcessor::UpdateWorldGrid()
{
	/*
	for (int y = 0; y < localCostmap.info.height; y++)
	{
		for (int x = 0; x < localCostmap.info.width; x++)
		{
			int localGridIndex = TF::Grid2Array(localCostmap.info.width, x, y);
			float tmpx;
			float tmpy;
			//格子上の点がどこに移動するか
			CoordinateTransform::Grid2Position(tmpx, tmpy, localCostmap.offset.x, localCostmap.offset.y, x, y, localCostmap.info.resolution);
			float localGridPosX = tmpx;
			float localGridPosY = tmpy;
			//Localでの格子上の点はワールドでどこに位置するのか
			Vector3 pos=TF::Local2WorldPoint(w2lPoint.position,localPoint,rotZ);
			int worldGridIndexX = x;
			int worldGridIndexY = y;
			//ワールドでの位置はワールドでのグリッド位置のどこに位置するのか。
			CoordinateTransform::Position2Grid(worldGridIndexX, worldGridIndexY,
											   globalCostmap.centerX, globalCostmap.centerY,
											   tmpx, tmpy, globalCostmap.resolution);
			int worldGridIndex = TF::Grid2Array(globalCostmap.width, worldGridIndexX, worldGridIndexY);
			//cout << "gx" << worldGridIndexX << "gy" << worldGridIndexY << "\n";
			int localVal = Math::Threashold(Data[localGridIndex], 0, 100);
			if (localVal > 3)
			{
				worldData[worldGridIndex] = localVal;
			}
		}
	}
	*/
}

void OccupancyGridProcessor::InitializeGrid(OccupancyGrid &costmap)
{
	costmap.Data.assign(costmap.info.width * costmap.info.height, 0);
}

void OccupancyGridProcessor::UpdateOccupancyGrid(OccupancyGrid &costmap, vector<Vector3> &objects)
{
	if (isDebug)
	{
		cout << "local objects size" << objects.size() << "\n";
	}
	//限定されたオブジェクトにたいして
	for (Vector3 obj : objects)
	{
		Vector2 position = {obj.x, obj.y};
		Vector2i grid = TF::Position2Grid(position, costmap.offset, costmap.info.resolution);
		Costmap::CalculateHeightMap(costmap, grid.x, grid.y);
	}
}

//先にはじくのが一般的のため一つひとつのオブジェクトに対する処理とする
// x,yはローカル座標

vector<Vector3> OccupancyGridProcessor::LimmitObjects2Range(OccupancyGrid &costmap, DetectedObjectArray &objectsWorld)
{
	if (isDebug)
	{
		cout << "world objects size" << objectsWorld.objects.size() << "\n";
	}
	vector<Vector3> localObjects = {};
	for (int i = 0; i < objectsWorld.objects.size(); i++)
	{
		//ここでワールドからローカルに変換する position euler.zを使用する
		float objx = objectsWorld.objects[i].pose.position.x;
		float objy = objectsWorld.objects[i].pose.position.y;

		Vector3 worldPoint = {objx, objy, 0.f};
		Vector3 localPoint = TF::World2LocalVector(
			costmap.info.origin,
			worldPoint);

		float localObjX = localPoint.x;
		float localObjY = localPoint.y;
		//ローカルの座標系に変換されたオブジェクトを使用する。
		if (gridRange.r < localObjX && localObjX < gridRange.r &&
			gridRange.r < localObjY && localObjY < gridRange.r)
		{
			localObjects.emplace_back(localObjX, localObjY, 0.f);
		}
	}
	//ここですべてのオブジェクトをローカルに格納し終わる
	return localObjects;
}

void OccupancyGridProcessor::Start()
{
	//gridの数は必ず奇数
	//Hは必ず５の倍数+1
	//0含む左から何番目
	//0含む下から何番目
	//time_t time = time(NULL);
	localCostmap.header = {0, time(NULL), "local costmap", 0.f};
	localCostmap.info = {
		time(NULL),
		local_costmap::resolution,
		local_costmap::width,
		local_costmap::height,
		local_costmap::origin};

	localCostmap.offset = {(localCostmap.info.width - 1) / 10 + 1,
						   (localCostmap.info.height - 1) / 2 + 1};

	//worldData.reset(new int(201 * 201));
	//xは  localCostmap.offset.x=localCostmap.info.width/2のため左右対称
	gridRange.r = (localCostmap.info.width - 1) * localCostmap.info.resolution / 2;
	gridRange.r = -(localCostmap.info.width - 1) * localCostmap.info.resolution / 2;
	//y方向は多少のオフセットTyを用いているため位置がずれる
	gridRange.r = (localCostmap.info.height - localCostmap.offset.y) * localCostmap.info.resolution;
	gridRange.r = -(localCostmap.offset.y - 1) * localCostmap.info.resolution;

	NavigationMsgs::instance()->localCostmap = this->localCostmap;
}

void OccupancyGridProcessor::GetState()
{
	localCostmap.info.origin = VehicleStateMsgs::instance()->geometry.pose;
	detectedObjectArray = SensorMsgs::instance()->detectedObjectArray;
}

void OccupancyGridProcessor::SetState()
{
	NavigationMsgs::instance()->localCostmap = this->localCostmap;
	//SensorMsgs::instance()->objects.clear();
	NavigationMsgs::instance()->localPath.header.seq++;
	if (isDebug)
	{
		localCostmap.Debug();
	}
}

void OccupancyGridProcessor::SpinOnce()
{
	InitializeGrid(localCostmap);
	this->objectsLocal = LimmitObjects2Range(localCostmap, detectedObjectArray);
	UpdateOccupancyGrid(localCostmap, objectsLocal);
	//UpdateWorldGrid();
}

//===============================================================================================================================================================================

//void AOccupancyGrid::

void AOccupancyGrid::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void AOccupancyGrid::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	if (SensorMsgs::instance()->detectedObjectArray.header.seq > 0)
	{
		Print::Start("LocalCostmap");
		oc->Update();
		Print::End("LocalCostmap");
		this->Update();
	}
}

void AOccupancyGrid::Update()
{
	//DrawGridTick(NavigationMsgs::position);
	UnrealDrawer::DrawLocalGrid(
		GetWorld(),
		NavigationMsgs::instance()->localCostmap);
}

// Called when the game starts or when spawned
// Sets default values
AOccupancyGrid::AOccupancyGrid()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}
AOccupancyGrid::~AOccupancyGrid()
{
}
