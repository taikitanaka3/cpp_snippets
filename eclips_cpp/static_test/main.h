/*
 * main.h
 *
 *  Created on: Dec 5, 2019
 *      Author: tanakasan
 */

#ifndef MAIN_H_
#define MAIN_H_

#define _USE_MATH_DEFINES

#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>
using std::cout;
using std::endl;
using std::vector;

struct occg{
	static float x;

};
class constc{
public:
	const int i;
};

struct Point
{
	float X;
	float Y;
	float Z;
	Point(float x = 0.f, float y = 0.f, float z = 0.f)
	{
		X = x;
		Y = y;
		Z = z;
	}
};

struct Pose
{
	static Point point;
	static float angle;
private:
};

struct PoseArray
{
	static vector<Pose> pose;
private:
	//PoseArray();
};


struct Waypoints
{
	static vector<PoseArray> waypoints;
	static Point target;
	static float angle;

private:
	Waypoints();
};


#endif /* MAIN_H_ */
