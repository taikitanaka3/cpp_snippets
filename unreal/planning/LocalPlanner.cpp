// Fill out your copyright notice in the Description page of Project Settings.

#include "LocalPlanner.h"

#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/math_define.h"
#include "msgs/ros_msgs.h"
#include "msgs/auto_msgs.h"
#include "msgs/trajectory_msgs.h"
#include <iostream>
#include <time.h>
#include <vector>
#include <math.h>
using std::cout;
using std::endl;
using std::vector;

#include "Math/Color.h"
#include "define/unreal_define.h"
#include "DrawDebugHelpers.h"

// Sets default values
ALocalPlanner::ALocalPlanner()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALocalPlanner::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void ALocalPlanner::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	if (NavigationMsgs::instance()->localPath.header.seq > 0)
	{
		Print::Start("Local Planner");
		//lp->Update();
		//dwa->Update();
		sl->Update();
		Print::End("Local Planner");
		if (isDebug)
		{
			this->Update();
		}
	}
}

void ALocalPlanner::Update()
{
	//UnrealDrawer::DrawLoci(GetWorld(),
	//					   TrajectoryMsgs::instance()->loci,
	//					   FColor::Black,
	//					   2.6f,
	//					   10.f,
	//					   0.1f);
	//

	UnrealDrawer::DrawPoint(GetWorld(),
							NavigationMsgs::instance()->localPath.target,
							FColor::Orange,
							1.6f,
							10.f,
							-1.f);

	UnrealDrawer::DrawLine(GetWorld(),
						   VehicleStateMsgs::instance()->geometry.pose.position,
						   NavigationMsgs::instance()->localPath.target,
						   FColor::Orange,
						   1.6f,
						   0.05f,
						   1.f);
}