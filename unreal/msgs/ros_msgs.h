#pragma once
#include "../design/SingletonPattern.h"
#include "../define/core_define.h"
#include "../define/sensor_define.h"
#include "../define/geom_define.h"
#include "../define/nav_define.h"
#include "../define/ackermann_define.h"

class AckermannMsgs : public Singleton<AckermannMsgs>
{
    friend class Singleton<AckermannMsgs>;
    AckermannMsgs(){};
    AckermannMsgs(AckermannMsgs const &){};

public:
    AckermannDrive ackermannDrive = {};
};

class FakeLocalizationMsgs : public Singleton<FakeLocalizationMsgs>
{
    friend class Singleton<FakeLocalizationMsgs>;
    FakeLocalizationMsgs(){};
    FakeLocalizationMsgs(FakeLocalizationMsgs const &){};

public:
    Odmetry odmetry;
    IMU imu;
};

class NavigationMsgs : public Singleton<NavigationMsgs>
{
    friend class Singleton<NavigationMsgs>;
    NavigationMsgs(){};
    NavigationMsgs(NavigationMsgs const &){};

public:
    OccupancyGrid globalCostmap;
    OccupancyGrid localCostmap;
    GetPlan getPlan;
    Vector3 navGoal;
    Path globalPath;
    Path localPath;
    Odmetry odmetry;
};
