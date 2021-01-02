#pragma once
#define _USE_MATH_DEFINES
#include "core_define.h"
#include "geom_define.h"
#include <iostream>
#include <vector>
#include <map>
#include <math.h>
#include <time.h>
#include <array>

struct ControlCommand
{
    float linear_velocity;
    float linear_acceleration; // #m/s^2
    float steering_angle;
    ControlCommand(const float linear_velocity = 0.f,
                   const float linear_acceleration = 0.f,
                   const float steering_angle = 0.f)
    {
        this->linear_velocity = linear_velocity;
        this->linear_acceleration = linear_acceleration;
        this->steering_angle = steering_angle;
    }
    void Debug()
    {
        cout << "/---frame:" << "ctrl_cmd"<<"---/"<<endl;
        cout << "    linear_velocity:" << linear_velocity<< endl;
    }
};
struct AccelCmd
{
    Header header = {};
    float throttle = 0.f;
    AccelCmd(const Header &header = Header(),
             const float throttle = 0.f)
    {
        this->header = header;
        this->throttle = throttle;
    }
    void Debug()
    {
        header.Debug();
        cout << "    throttle" << throttle << endl;
    }
};
struct SteerCmd
{
    Header header = {};
    float steering = 0.f;
    SteerCmd(const Header &header = Header(),
             const float steering = 0.f)
    {
        this->header = header;
        this->steering = steering;
    }
    void Debug()
    {
        header.Debug();
        cout << "    steering" << steering << endl;
    }
};

struct BrakeCmd
{
    Header header = {};
    float brake = 0.f;
    BrakeCmd(const Header &header = Header(),
             const float brake = 0.f)
    {
        this->header = header;
        this->brake = brake;
    }
    void Debug()
    {
        header.Debug();
        cout << "    brake" << brake << endl;
    }
};