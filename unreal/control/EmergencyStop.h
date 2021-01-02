// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "design/TemplateMethod.h"
#include "define/sensor_define.h"
#include "define/ctrl_define.h"


class EmergencyStop : public TemplateMethod
{
	
	bool isDebug=true;
	void GetState() override;
	void SpinOnce() override;
	void SetState() override;
	void Start() override;
	UltraSonic ultraSonic;
	BrakeCmd brakeCmd;
	AccelCmd accelCmd;

public:
	EmergencyStop();
	~EmergencyStop();
};


