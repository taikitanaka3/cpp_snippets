// Fill out your copyright notice in the Description page of Project Settings.

#include "GlobalCostmap.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/math_define.h"
#include "define/tf_define.h"
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
#include "define/unreal_define.h"
#include "DrawDebugHelpers.h"
#include "Math/Color.h"

GlobalCostmap::GlobalCostmap()
{
	cout << "create Globalcostmap" << endl;
}

void GlobalCostmap::InitializeGrid(OccupancyGrid &costmap)
{
	costmap.Data.assign(costmap.info.width * costmap.info.height, 0);
}

void GlobalCostmap::UpdateOccupancyGrid(OccupancyGrid &costmap, DetectedObjectArray &objectsWorld)
{
	if (isDebug)
	{
		cout << "objects size" << objectsWorld.objects.size() << "\n";
	}
	//限定されたオブジェクトにたいして
	for (DetectedObject obj : objectsWorld.objects)
	{
		Pose objPose = obj.pose;
		Vector2 position = {objPose.position.x, objPose.position.y};
		Vector2i grid = TF::Position2Grid(position,costmap.offset, costmap.info.resolution);
		Costmap::CalculateHeightMap(costmap, grid.x, grid.y);
	}
}
//まあ、時間あったら最適化するよねｗ
void GlobalCostmap::CalculateHeightMap(OccupancyGrid &costmap, int x, int y)
{
	int xmin = Math::Saturation(x - int(clearanceSize / costmap.info.resolution), 0, costmap.info.width);
	int xmax = Math::Saturation(x + int(clearanceSize / costmap.info.resolution), 0, costmap.info.width);
	int ymin = Math::Saturation(y - int(clearanceSize / costmap.info.resolution), 0, costmap.info.height);
	int ymax = Math::Saturation(y + int(clearanceSize / costmap.info.resolution), 0, costmap.info.height);

	for (int j = ymin; j < ymax; j++)
	{
		for (int i = xmin; i < xmax; i++)
		{
			int index = TF::Grid2Array(costmap.info.width, i, j);
			//if(data<1 is ok too!)
			if (costmap.Data[index] == 0)
			{
				int newData = costmap.Data[index];
				float dist = hypotf(x - i, y - j);
				if (dist < crashSize)
				{
					newData = limmitIndex - 1;
				}
				else if (dist < clearanceSize)
				{
					newData = 1 + int(float(warningIndex / dist));
				}
				else
				{
				}
				costmap.Data[index] = newData;
			}
		}
	}
	//objectcの位置
	int objectIndex = TF::Grid2Array(costmap.info.width, x, y);
	costmap.Data[objectIndex] = 100;
}
void GlobalCostmap::SetState() {}

void GlobalCostmap::GetState() {}

void GlobalCostmap::Start()
{
	globalCostmap.header = {0, time(NULL), "Global Costmap", 0.f};
	globalCostmap.info = {time(NULL), 2.5f, 201, 201, Pose()};
	globalCostmap.offset = {(globalCostmap.info.width - 1) / 2,
							(globalCostmap.info.height - 1) / 2};
	//Initialize global costmap at Start
	InitializeGrid(globalCostmap);
	detectedObjectArray = SensorMsgs::instance()->detectedObjectArray;
	if (isDebug)
	{
		cout << "world objects size" << detectedObjectArray.objects.size() << "\n";
	}
	UpdateOccupancyGrid(globalCostmap, detectedObjectArray);

	if (isDebug)
	{
		//cout << "height" << globalCostmap.info.height << "Data" << globalCostmap.Data[1];
		for (const auto &data : globalCostmap.Data)
		{
			//cout << "D" << data;
		}
	}
	NavigationMsgs::instance()->globalCostmap = this->globalCostmap;
}

void GlobalCostmap::SpinOnce()
{
}

// Sets default values
AGlobalCostmap::AGlobalCostmap()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void AGlobalCostmap::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void AGlobalCostmap::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	if (SensorMsgs::instance()->detectedObjectArray.header.seq > 0)
	{
		gc->Update();
		if (!bInitialized)
		{
			this->Start();
			bInitialized = true;
			cout << "Init Occ \n\n";
		}
	}
}

void AGlobalCostmap::Start()
{
	UnrealDrawer::DrawWorldGrid(GetWorld(), NavigationMsgs::instance()->globalCostmap);
}
