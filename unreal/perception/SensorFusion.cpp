// Fill out your copyright notice in the Description page of Project Settings.

#include "SensorFusion.h"
#include "define/math_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/unreal_define.h"
#include "msgs/auto_msgs.h"
#include "msgs/ros_msgs.h"
#include <time.h>
#include <math.h>

SensorFusion::SensorFusion()
{
	cout << "created SensorFusion" << endl;
}

SensorFusion::~SensorFusion()
{
	cout << "del SensorFusion" << endl;
}

void SensorFusion::SetState()
{
	//VehicleStateMsgs::instance()->vehicleState.header++;
	VehicleStateMsgs::instance()->geometry.pose = this->fakeOdmetry.pose.pose;
	VehicleStateMsgs::instance()->geometry.twist = this->fakeOdmetry.twist.twist;

}

void SensorFusion::GetState()
{
	this->fakeOdmetry = FakeLocalizationMsgs::instance()->odmetry;
	this->kineOdmetry = NavigationMsgs::instance()->odmetry;
}

void SensorFusion::Start()
{
}

void SensorFusion::SpinOnce()
{
}

// Sets default values
ASensorFusion::ASensorFusion()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ASensorFusion::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void ASensorFusion::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	if (NavigationMsgs::instance()->odmetry.header.seq > 0 && FakeLocalizationMsgs::instance()->odmetry.header.seq > 0)
	{
		sf->Update();
	}
}
