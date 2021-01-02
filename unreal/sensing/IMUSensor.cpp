// Fill out your copyright notice in the Description page of Project Settings.

#include "IMUSensor.h"
#include "define/math_define.h"
#include "define/unreal_define.h"
#include "define/sensor_define.h"
#include "msgs/ros_msgs.h"
#include "msgs/auto_msgs.h"
#include "msgs/sensor_msgs.h"
#include <time.h>
#include <math.h>
#include <vector>


#include "Math/Color.h"

void AIMUSensor::Start()
{
	imu.header = {0, time(NULL), "imu", 0.f};
}
void AIMUSensor::GetState()
{
	this->imu.linear_acceleration = FakeLocalizationMsgs::instance()->imu.linear_acceleration;
	this->imu.angular_velocity = FakeLocalizationMsgs::instance()->imu.angular_velocity;
}

void AIMUSensor::SpinOnce()
{
	//lienarに対して
	//void odmetry.twist.linear=Smooth(this->imu.linear_acceleration);
	//void this->odm.twist.linear=Integrate(this->imu.linear_acceleration;

	//angularに対して
	//void Smooth(this->imu.angular_velocity);
	//void this->imu.angular_velocity=Integrate(this->imu.angular_velocity);
}
void AIMUSensor::SetState()
{
	imu.header.seq++;
	SensorMsgs::instance()->imu=this->imu;
	if (isActive)
	{
		NavigationMsgs::instance()->odmetry.twist.twist.linear = this->odm.twist.twist.linear;
		NavigationMsgs::instance()->odmetry.twist.twist.angular.z = this->imu.angular_velocity.z;
	}
}
// Sets default values
AIMUSensor::AIMUSensor()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	cout << "create IMU sensor" << endl;
}

// Called when the game starts or when spawned
void AIMUSensor::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void AIMUSensor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	this->Update();
}
