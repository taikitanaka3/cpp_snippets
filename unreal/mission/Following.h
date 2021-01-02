// /*copy rigth 2018 Taiki Tanaka Under GPLlv3 License*/

#pragma once
#include "design/TemplateMethod.h"
#include "define/sensor_define.h"
#include "define/ctrl_define.h"
#include "define/ackermann_define.h"


/**
 * 
 */
class Following : public TemplateMethod
{
	bool isDebug = true;
	void GetState() override;
	void SpinOnce() override;
	void SetState() override;
	void Start() override{};
	//
	float Follow(float dist);
	UltraSonic ultraSonic;
	BrakeCmd brakeCmd;
	AccelCmd accelCmd;
	VehicleState vehicleState;
	AckermannDrive ackermannDrive;
	ControlCommand controlCommand;
	float distOld;
	float safetyDistance;

public:
	Following();
	~Following();
};
