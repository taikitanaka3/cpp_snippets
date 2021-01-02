// Fill out your copyright notice in the Description page of Project Settings.

#include "Recognition.h"
#include "define/geom_define.h"
#include "define/math_define.h"
#include "define/nav_define.h"
#include "define/tf_define.h"
#include "define/trajectory_define.h"
#include "define/core_define.h"
#include "msgs/auto_msgs.h"
#include "msgs/ros_msgs.h"
#include "msgs/sensor_msgs.h"
#include "msgs/trajectory_msgs.h"
#include <algorithm>
#include <math.h>
#include <time.h>
#include <iostream>

using namespace std;


/*depend PolarVector
 * in  : polar vector
 * out : index
 * */
int Recognition::CClockwiseClustering(PolarVector pvec) {
	const float RANGE = 30.f;//deg
	const float OFFSET = M_PI/12.f;
	float rad = pvec.th;
	rad = rad + OFFSET;
	int rot = (int) (rad / M_PI);
	rot = rot - rot % 2;
	rad = rad - (float) abs(rot) * M_PI; //回転対応
	if (rad < 0.f) rad = PI_2 + rad;
	int index = static_cast<int>(rad * RAD2DEG / RANGE);
	if(index>11) {
		cout << "index over" << endl;
		index=11;
	}
	return index;
}

vector<float> Recognition::Scan()
{
	vector<float> dists(12, 10000.f);

	for (const DetectedObject otherObj:detectedObjectArray.objects) {
		PolarVector polarVector = TF::World2LocalPolarVector(geometry.pose, otherObj.pose.position);
		int index = CClockwiseClustering(polarVector);
		float val = dists[index];
		if (val > polarVector.r) dists[index] = polarVector.r;
	}
	for (int i = 0; i < 12; i++) {
		cout << i << ":" << dists[i] << "\n";
	}
	return dists;
}

Recognition::Recognition()
{
	cout << "create recognition " << endl;
}

Recognition::~Recognition()
{
}

void Recognition::Start()
{
	scanData.header = {0, time(NULL), "recognition", 0.f};
}
void Recognition::GetState()
{
	this->detectedObjectArray = SensorMsgs::instance()->detectedObjectArray;
	this->geometry = VehicleStateMsgs::instance()->geometry;
}
void Recognition::SpinOnce()
{
	scanData.dists = Scan();
}
void Recognition::SetState()
{
	scanData.header.seq++;
	SensorMsgs::instance()->scandata = this->scanData;
	if (isDebug)
	{
		scanData.Debug();
	}
}

