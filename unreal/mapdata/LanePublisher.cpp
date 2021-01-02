// Fill out your copyright notice in the Description page of Project Settings.

#include "LanePublisher.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/waypoint_define.h"
#include "msgs/ros_msgs.h"
#include "msgs/auto_msgs.h"
#include "config/waypoint_config.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <map>
using std::cout;
using std::endl;
using std::map;
using std::vector;

#include "define/unreal_define.h"
#include "DrawDebugHelpers.h"
#include "Math/Color.h"

LanePublisher::LanePublisher()
{
	cout << "create lane publisher" << endl;
}

LanePublisher::~LanePublisher()
{
	cout << "del lane publisher" << endl;
}

Pose LanePublisher::MakeStraightWaypoints(vector<Waypoint> &waypoints, Pose initialPose, float d, float len)
{
	Pose pose={};
	Twist twist={};
	float speed = 10.f;
	int num = int(len / d);
	Waypoint waypoint;
	DTLane dtlane = {};
	for (int i = 0; i < num; i++)
	{
		float x = initialPose.position.x + d * cos(initialPose.euler.z) * i;
		float y = initialPose.position.y + d * sin(initialPose.euler.z) * i;
		float z = 0.f;
		float th = atan2(y - pose.position.y, x - pose.position.x);
		pose.position = {x, y, z};
		pose.euler = {0.f, 0.f, th};
		float xd = speed * cos(th);
		float yd = speed * sin(th);
		twist.linear = {xd, yd, 0.f};
		waypoint.pose.pose = pose;
		waypoint.twist.twist = twist;
		waypoints.emplace_back(waypoint);
	}
	return pose;
}

Lane LanePublisher::MakeRightLane()
{
	Lane lane = {};
	lane.header = {1, 0, "lanes[right] array", 0.f};
	Waypoint waypoint = {};
	waypoint.pose.pose.position = {-3.0f, -60.f, 0.f};
	waypoint.pose.pose.euler = {0.f, 0.f, M_PI_2};
	waypoint.pose.pose = MakeStraightWaypoints(lane.waypoints, waypoint.pose.pose, 1.0f, 120.0f);
	return lane;
}

void LanePublisher::GetState() {}
void LanePublisher::SetState() {}
void LanePublisher::SpinOnce() {}
void LanePublisher::Start()
{
	LaneArrayMsgs::instance()->lane_array.id = 0;
	LaneArrayMsgs::instance()->lane_array.lanes[lane_config::lane_index::RIGHT] = MakeRightLane();
	//LaneArrayMsgs::instance()->lane_array.lanes[lane_config::lane_index::LEFT].waypoints = LeftLane();
}
//=======================================================================================================

// Called when the game starts or when spawned
void ALanePublisher::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void ALanePublisher::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	if (!bInitialized)
	{
		way->Update();
		this->Update();
		bInitialized = true;
	}
}

// Sets default values
ALanePublisher::ALanePublisher()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

void ALanePublisher::Update()
{
	UnrealDrawer::DrawLane(
		GetWorld(),
		LaneArrayMsgs::instance()->lane_array.lanes[lane_config::lane_index::RIGHT],
		FColor::Blue, 1.7f, 0.1f, 1000.f);
}

ALanePublisher::~ALanePublisher()
{
	cout
		<< "del Awaypoint class";
}
