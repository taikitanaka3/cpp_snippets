// Fill out your copyright notice in the Description page of Project Settings.

#include "MoveBase.h"
#include "config/control_config.h"
#include "config/vehicle_config.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/waypoint_define.h"
#include "define/unreal_define.h"
#include "define/math_define.h"
#include "define/ctrl_define.h"
#include "msgs/ros_msgs.h"
#include "msgs/auto_msgs.h"

#include <time.h>
#include <iostream>
using std::cout;
using std::endl;

#include "DrawDebugHelpers.h"
#include "Math/Color.h"
MoveBase::MoveBase()
{
	cout << "created Movebase" << endl;
}

MoveBase::~MoveBase()
{
	cout << "del Movebase " << endl;
}

float MoveBase::ThrottleControl(float current)
{
	float deltaTrottle = controlCommand.linear_velocity - current;
	float pThrottle = gain::throttle["P"] * deltaTrottle;
	float dThrottle = gain::throttle["D"] * (deltaTrottle - deltaThrottleOld) / dt;
	deltaThrottleOld = deltaTrottle;
	//cout << "tar"<< ctrl::target_speed <<"curr"<< current <<endl;
	return pThrottle + dThrottle;
}

// get steering input from waypoint target
float MoveBase::SteerControl()
{
	//assume d_old=0
	float deltaSteering = geometry.twist.linear.x * pos2Goal.y - geometry.twist.linear.y * pos2Goal.x;
	float pSteering = gain::steer["P"] * deltaSteering;
	float dSteering = gain::steer["D"] * (deltaSteering - deltaSteeringOld) / dt;
	deltaSteeringOld = deltaSteering;
	return pSteering + dSteering;
}

void MoveBase::Start()
{
	accelCmd.header = {0, time(NULL), "accel cmd", 0.f};
	steerCmd.header = {0, time(NULL), "steer cmd", 0.f};
	controlCommand.linear_velocity = ctrl::target_speed;
	//strategy pattern でAPIの変更
}

void MoveBase::GetState()
{

	geometry = VehicleStateMsgs::instance()->geometry;
	this->dt = VehicleStateMsgs::instance()->vehicleState.header.deltaTime;
	ackermannDrive = AckermannMsgs::instance()->ackermannDrive;
	goal = NavigationMsgs::instance()->getPlan.goal.position;
	controlCommand = VehicleCmdMsgs::instance()->controlCommand;
	//今回はLaneテストのため
	controlCommand.linear_velocity = ctrl::target_speed;
}
void MoveBase::SpinOnce()
{
	//strategy pattern でAPIの変更
	this->pos2Goal = {goal.x - geometry.pose.position.x,
					  goal.y - geometry.pose.position.y,
					  0.f};

	float steering = SteerControl();
	//LinearControl();
	//PurePursuitControl();
	float throttle = ThrottleControl(ackermannDrive.speed);

	steering = UnrealUnit::Unreal2ISOSteer(steering);
	steerCmd.steering = Math::Saturation(steering, -vehicle::limmit::STEERING_LIMMIT, vehicle::limmit::STEERING_LIMMIT);
	accelCmd.throttle = Math::Saturation(throttle, -vehicle::limmit::THROTTLE_LIMMIT, vehicle::limmit::THROTTLE_LIMMIT);

	accelCmd.header.seq++;
	steerCmd.header.seq++;
}
void MoveBase::SetState()
{
	VehicleCmdMsgs::instance()->accelCmd = accelCmd;
	VehicleCmdMsgs::instance()->steerCmd = steerCmd;
	//NavigationMsgs::instance()->setMotion = true;
	if (isDebug)
	{
		accelCmd.Debug();
		steerCmd.Debug();
		controlCommand.Debug();
	}
}

// Sets default values
AMoveBase::AMoveBase()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void AMoveBase::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void AMoveBase::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	if (NavigationMsgs::instance()->getPlan.header.seq > 0)
	{
		Print::Start("MoveBase");
		mv->Update();
		Print::End("MoveBase");
		Print::Start("EmergencyStop");
		eStop->Update();
		Print::End("EmergencyStop");
		if (isDebug)
		{
		}
	}
}
