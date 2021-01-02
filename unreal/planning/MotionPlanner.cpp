// Fill out your copyright notice in the Description page of Project Settings.
#include "MotionPlanner.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "msgs/ros_msgs.h"
#include "msgs/auto_msgs.h"
#include <iostream>
#include <math.h>
#include <vector>
using std::cout;
using std::endl;
using std::vector;
//=
#include "define/unreal_define.h"
#include "DrawDebugHelpers.h"
#include "Math/Color.h"

MotionPlanner::MotionPlanner()
{
	cout << "create motion planener " << endl;
}
MotionPlanner::~MotionPlanner()
{
	cout << "del motion planner" << endl;
}

/*これはMotionPlannerで対応しきれなかった
	緊急回避
	or
	緊急停止
なのでLocalPlannerではなくMotionPlanerにおいておく
ホントはRayCastの値を使用する.
*/
/*
int MotionPlanner::OverRide(int currentLane)
{

	Vector3 avoidPoint = {-3.0f, 0.0f, 0.0f};
	Vector3 changePoint = {-3.0f, -60.0f, 0.0f};

	bool bOverride = false;
	bool bLanechange = false;
	int newLane = currentLane;
	float dist = hypotf(avoidPoint.x - waypoint_target.x, avoidPoint.y - waypoint_target.y);
	if (0.5f < dist && dist < 10.f)
	{
		bOverride = true;
		bLanechange = true;
		newLane = lane_config::lane_index::RIGHT; //right
	}
	else if (!bOverride && bLanechange)
	{
		bLanechange = false;
		newLane = lane_config::lane_index::LEFT; //left
	}

	dist = hypotf(changePoint.x - waypoint_target.x, changePoint.y - waypoint_target.y);
	if (dist < 3.0f)
	{
		newLane = lane_config::lane_index::LEFT; //left
	}
	return newLane;
}*/

//===============================================
void MotionPlanner::Start()
{
	getPlan.header = {0, 0, "get plan", 0.f};
}

void MotionPlanner::GetState()
{
	this->localPath = NavigationMsgs::instance()->localPath;
	this->globalPath = NavigationMsgs::instance()->globalPath;
}

void MotionPlanner::SpinOnce()
{
	//前方注視モデル
	//LookAheadModel();
	//最近棒点探索
	//SearchNearestWaypoints(stare_pos.x, stare_pos.y);
	//はじめの一回はスキップ
	//newLaneIndex = Override(laneIndex);
	/*ここでLocalとGlobalの使い分け
	条件は２つ
		1.ぶつかるかどうかー＞OccupancyGridにより判定->Plan変更
		2.waypointから離れすぎているかどうか->RecoveryMotionへ遷移
	*/
	/// wrench method選択
	goal = globalPath.target;
	if (localPath.target.z > 0.1f)
	{
		//goal = localPath.target;
	}
	getPlan.goal.position = goal;
}

void MotionPlanner::SetState()
{
	getPlan.header.seq++;
	NavigationMsgs::instance()->getPlan = getPlan;
	if (isDebug)
	{
		Print::Start("Motion Planner");
		NavigationMsgs::instance()->getPlan.Debug();
		Print::Start("Motion Planner");
	}
}
//==================================================

// Sets default values
AMotionPlanner::AMotionPlanner()
{
	cout << "Created AMoionPlanner";
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void AMotionPlanner::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void AMotionPlanner::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	if (NavigationMsgs::instance()->globalPath.header.seq > 0)
	{
		mp->Update();
		if (isDebug)
			this->Update();
	}
	return;
}

void AMotionPlanner::Update()
{
	UnrealDrawer::DrawSphere(
		GetWorld(),
		NavigationMsgs::instance()->getPlan.goal.position,
		FColor::Green,
		1.7f,
		1.5f,
		-1.f);
	//UnrealDrawer::DrawLine(GetWorld(),
	//					   VehicleStateMsgs::instance()->geometry.pose.position,
	//					   NavigationMsgs::instance()->getPlan.goal.position,
	//					   FColor::FColor::Red,
	//					   1.7f,
	//					   2.f,
	//					   1.f);
}
