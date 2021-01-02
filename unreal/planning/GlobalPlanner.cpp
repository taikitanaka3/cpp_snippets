// Fill out your copyright notice in the Description page of Project Settings.

#include "GlobalPlanner.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/waypoint_define.h"
#include "msgs/auto_msgs.h"
#include "msgs/ros_msgs.h"
#include "config/waypoint_config.h"
#include <time.h>

#include "define/unreal_define.h"
#include "Math/Color.h"
#include "DrawDebugHelpers.h"

//================================
// Sets default values
AGlobalPlanner::AGlobalPlanner() {
    // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
    PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void AGlobalPlanner::BeginPlay() {
    Super::BeginPlay();
}

// Called every frame
void AGlobalPlanner::Tick(float DeltaTime) {
    Super::Tick(DeltaTime);

    if (LaneArrayMsgs::instance()->lane_array.lanes.size() > 0 &&
        VehicleStateMsgs::instance()->vehicleState.header.seq > 0) {
        //gp->Update();
        Print::Start("Global Planner");
        ws->Update();
        Print::End("Global Planner");
        this->Update();
    }
}

void AGlobalPlanner::Update() {
    int id = VehicleStateMsgs::instance()->vehicleLocation.lane_array_id;
    int index = VehicleStateMsgs::instance()->vehicleLocation.waypoint_index;
    Vector3 offset = {0.f, 0.f, 5.f};
    Vector3 arrowTo = LaneArrayMsgs::instance()->lane_array.lanes[id].waypoints[index].pose.pose.position;
    Vector3 arrowFrom = Vector3::Add(arrowTo, offset);

    UnrealDrawer::DrawArrow(GetWorld(), arrowFrom, arrowTo,FColor::Purple,0.f,0.1f);

    UnrealDrawer::DrawPath(GetWorld(),
                           NavigationMsgs::instance()->globalPath,
                           FColor::Purple,
                           0.3f,
                           0.2f,
                           -1.f);
    //UnrealDrawer::DrawPoint(GetWorld(),
    //						NavigationMsgs::instance()->globalPath.target,
    //						FColor::Purple,
    //						1.5f,
    //						10.f,
    //						-1.f);
    UnrealDrawer::DrawLine(GetWorld(),
                           VehicleStateMsgs::instance()->geometry.pose.position,
                           NavigationMsgs::instance()->globalPath.target,
                           FColor::FColor::Purple,
                           1.5f,
                           0.05f,
                           1.f);
}