// Fill out your copyright notice in the Description page of Project Settings.
#define _USE_MATH_DEFINES
#include "GNSSSensor.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/math_define.h"
#include "define/unreal_define.h"
#include "msgs/ros_msgs.h"
#include "msgs/auto_msgs.h"
#include <math.h>


#include "Math/Color.h"
#include "DrawDebugHelpers.h"

void AGNSSSensor::Start()
{
}

void AGNSSSensor::GetState()
{
	Vector3 position = UnrealUnit::Unreal2ISOVector(GetActorLocation());
	Vector3 euler = UnrealUnit::Unreal2ISOEuler(GetActorRotation());
	Vector3 velocity = UnrealUnit::Unreal2ISOVector(GetVelocity());

}

void AGNSSSensor::SpinOnce()
{
}

void AGNSSSensor::SetState()
{
	//store
	//NavigationMsgs::instance()->NavStatFix~~~
	//VehicleStateMsgs::instance()->geometry.pose = {position, euler.z};
	//VehicleStateMsgs::instance()->geometry.twist.linear = velocity;
	//VehicleStateMsgs::instance()->vehicleState.header.seq++;
}

// Sets default values
AGNSSSensor::AGNSSSensor()
{
	cout << "create gnss sensor" << endl;
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}
// Called when the game starts or when spawned
void AGNSSSensor::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void AGNSSSensor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	//this->dt = DeltaTime;
	this->Update();
	//DrawGNSSPoint(VehicleStateMsgs::instance()->geometry.pose.position);
	//cout << "GNSSX" << position.x << "GNSSY" << position.y << endl;
}

void AGNSSSensor::DrawGNSSPoint(const Vector3 &pos)
{
	FVector tmp = UnrealUnit::ISO2UnrealFVector(pos,1.5f);
	DrawDebugPoint(
		GetWorld(),
		tmp,
		3.f,
		FColor(155, 0, 255),
		false,
		0.1f);
}