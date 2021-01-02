#pragma once
#include "../define/ctrl_define.h"
#include "../define/waypoint_define.h"
#include "../design/SingletonPattern.h"
#include <iostream>
#include <vector>
#include <map>
#include <math.h>
#include <time.h>
#include <array>

class VehicleCmdMsgs : public Singleton<VehicleCmdMsgs>
{
    friend class Singleton<VehicleCmdMsgs>;
    VehicleCmdMsgs(){};
    VehicleCmdMsgs(VehicleCmdMsgs const &){};

public:
    AccelCmd accelCmd = {};
    SteerCmd steerCmd = {};
    BrakeCmd brakeCmd = {};
    ControlCommand controlCommand = {};
};

class VehicleStateMsgs : public Singleton<VehicleStateMsgs>
{
    friend class Singleton<VehicleStateMsgs>;
    VehicleStateMsgs(){};
    VehicleStateMsgs(VehicleStateMsgs const &){};

public:
    Geometry geometry = {};
    VehicleState vehicleState = {};
    VehicleLocation vehicleLocation = {};
};

class LaneArrayMsgs : public Singleton<LaneArrayMsgs>
{
    friend class Singleton<LaneArrayMsgs>;
    LaneArrayMsgs(){};
    LaneArrayMsgs(LaneArrayMsgs const &){};

public:
    LaneArray lane_array;
};