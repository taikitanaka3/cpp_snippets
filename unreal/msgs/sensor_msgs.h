#pragma once
#include "../define/core_define.h"
#include "../define/geom_define.h"
#include "../define/nav_define.h"
#include "../define/sensor_define.h"
#include "../design/SingletonPattern.h"

class SensorMsgs : public Singleton<SensorMsgs>
{
    friend class Singleton<SensorMsgs>;
    SensorMsgs(){};
    SensorMsgs(SensorMsgs const &){};
    ~SensorMsgs(){};

public:
    IMU imu = {};
    ObjPose objPose;
    DetectedObjectArray detectedObjectArray;
    UltraSonic ultrasonic;
    ScanData scandata;
};
