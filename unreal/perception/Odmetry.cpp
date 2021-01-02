// Fill out your copyright notice in the Description page of Project Settings.

#include "Odmetry.h"
#include "define/math_define.h"
#include "define/unreal_define.h"
#include "msgs/auto_msgs.h"
#include "msgs/ros_msgs.h"
#include <time.h>
#include <math.h>
#include <vector>

OdmetrySource::OdmetrySource()
{
	cout << "create odmetry" << endl;
}

OdmetrySource::~OdmetrySource()
{
	cout << "del odmetry" << endl;
}

void Estimate(const Accel &accel, Twist &twist, Pose &pose) {
	float dt = 0.02f;
	twist.linear.x = accel.linear.x * dt;
	twist.linear.y = accel.linear.y * dt;
	twist.linear.z = accel.linear.z * dt;
	twist.angular.z = accel.angular.z * dt;

	pose.position.x = twist.linear.x * dt;
	pose.position.y = twist.linear.y * dt;
	pose.position.z = twist.linear.z * dt;
	pose.euler.z = accel.angular.z * dt;

	return;
}

void OdmetrySource::Estimate(Pose &pose, Twist &twist)
{
	float angle = pose.euler.z;
	float phi = ackermannDrive.steering_angle;
	float v1 = GeometricCalc::TwistPose2Velocity(twist, pose);
	float v2 = ackermannDrive.steering_angle_velocity;
	float dt = odmetry.header.deltaTime;
	const float l = 2.4f;
	pose.position.x += dt * v1 * cos(angle);
	pose.position.y += dt * v1 * sin(angle);
	pose.euler.z += dt * v1 * tan(phi) / l;
	return;
}

vector<float> OdmetrySource::SetCovarianceMatrix(vector<float> data)
{

	for (int i = 0; i < xt.size(); i++)
	{
		xt[i].emplace_back(data[i]);
		xt[i].erase(xt[i].begin());
	}
	return Math::SetCovarianceMatrix(xt);
}

void OdmetrySource::Start()
{
	this->odmetry.child_frame_id = "vehicle";
	odmetry.header = {0, time(NULL), "odmetry", 0.f};
	odmetry.pose.pose = NavigationMsgs::instance()->odmetry.pose.pose;
	odmetry.pose.covariance.assign(odmetry_sorce::COVARIANCE_DIMENSION * odmetry_sorce::COVARIANCE_DIMENSION, 0.f);
}

void OdmetrySource::GetState()
{
	this->ackermannDrive = AckermannMsgs::instance()->ackermannDrive;
	this->odmetry.twist.twist = VehicleStateMsgs::instance()->geometry.twist;
	this->odmetry.pose.pose = NavigationMsgs::instance()->odmetry.pose.pose;
	this->odmetry.header.deltaTime = NavigationMsgs::instance()->odmetry.header.deltaTime;
}

void OdmetrySource::SpinOnce()
{
	Estimate(odmetry.pose.pose, odmetry.twist.twist);
	odmetry.pose.covariance = SetCovarianceMatrix({odmetry.pose.pose.position.x,
												   odmetry.pose.pose.position.y,
												   odmetry.pose.pose.euler.z});
	odmetry.header.seq++;
}

void OdmetrySource::SetState()
{
	NavigationMsgs::instance()->odmetry = this->odmetry;
	if (isDebug)
	{
		NavigationMsgs::instance()->odmetry.Debug();
	}
}

// Sets default values
AOdmetry::AOdmetry()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void AOdmetry::BeginPlay()
{
	Super::BeginPlay();
}

void AOdmetry::Initialize()
{
	Vector3 position = UnrealUnit::Unreal2ISOVector(GetActorLocation());
	Vector3 euler = UnrealUnit::Unreal2ISOEuler(GetActorRotation());
	Vector3 velocity = UnrealUnit::Unreal2ISOVector(GetVelocity());

	NavigationMsgs::instance()->odmetry.pose.pose = {position, euler};
}

// Called every frame
void AOdmetry::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	if (VehicleStateMsgs::instance()->vehicleState.header.seq > 0)
	{
		NavigationMsgs::instance()->odmetry.header.deltaTime = DeltaTime;
		if (NavigationMsgs::instance()->odmetry.header.seq % odmetry_sorce::RESET_FREQUENCY == 0)
		{
			this->Initialize();
		}

		Print::Start("Odmetry");
		odm->Update();
		Print::End("Odmetry");
		UnrealDrawer::DrawPoint(
			GetWorld(),
			NavigationMsgs::instance()->odmetry.pose.pose.position,
			FColor::White,
			2.5f,
			2.f,
			1.f);
	}
}
