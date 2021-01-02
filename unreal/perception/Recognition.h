// Fill out your copyright notice in the Description page of Project Settings.
#pragma once
#define _USE_MATH_DEFINES
#include "design/TemplateMethod.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/sensor_define.h"
#include "define/trajectory_define.h"
#include "define/ackermann_define.h"
#include <iostream>
#include <vector>
#include <map>
#include <math.h>
#include <time.h>
#include <array>
using std::array;
using std::cout;
using std::endl;
using std::map;
using std::vector;

class Recognition : public TemplateMethod
{
private:
	void GetState() override;
	void SpinOnce() override;
	void SetState() override;
	void Start() override;
    bool isDebug = true;
	float dt;
    
    vector<float> Scan();
	int CClockwiseClustering(PolarVector pvec);

    Geometry geometry;
	AckermannDrive ackermannDrive = {};
	DetectedObjectArray detectedObjectArray;
    ScanData scanData;
	
public:
	Recognition();
	~Recognition();
};
