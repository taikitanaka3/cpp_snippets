// Fill out your copyright notice in the Description page of Project Settings.

#include "EmergencyStop.h"
#include "config/vehicle_config.h"
#include "define/waypoint_define.h"
#include "define/math_define.h"
#include "msgs/ros_msgs.h"
#include "msgs/auto_msgs.h"
#include "msgs/sensor_msgs.h"

#include <time.h>
#include <iostream>
using std::cout;
using std::endl;

EmergencyStop::EmergencyStop()
{
    cout << "Emergency Stop" << endl;
}

EmergencyStop::~EmergencyStop()
{
    cout << "del Emergency Stop" << endl;
}

void EmergencyStop::Start()
{
}
void EmergencyStop::GetState()
{
    this->ultraSonic = SensorMsgs::instance()->ultrasonic;
    this->accelCmd = VehicleCmdMsgs::instance()->accelCmd;
}

void EmergencyStop::SpinOnce()
{
    float dist = ultraSonic.dist;
    brakeCmd.brake = 0.0f;
    if (safety::distance["Detected"] > dist && dist > safety::distance["Warning"])
    {
        cout << "Detected" << endl;
    }
    else if (safety::distance["Warning"] > dist && dist > safety::distance["Danger"])
    {
        //accelCmd.throttle *= 0.9f;
        cout << "Warning" << endl;
    }
    else if (safety::distance["Danger"] > dist&& dist > safety::distance["Crash"])
    {
        brakeCmd.brake = 0.5f;
        //accelCmd.throttle *= 0.0f;
        cout << "Danger" << endl;
    }
    else if (safety::distance["Crash"]>dist)
    {
        brakeCmd.brake = 1.0f;
        //accelCmd.throttle *= 0.0f;
        cout << "Crash" << endl;
    }
    else{
        cout << "No Obstacles" << endl;
    }
}

void EmergencyStop::SetState()
{
    VehicleCmdMsgs::instance()->brakeCmd = this->brakeCmd;
    VehicleCmdMsgs::instance()->accelCmd = this->accelCmd;
    if (isDebug)
    {
        brakeCmd.Debug();
    }
}
