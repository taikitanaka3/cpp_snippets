#pragma once
#define _USE_MATH_DEFINES
#include "core_define.h"
#include "math_define.h"
#include <iostream>
#include <vector>
#include <map>
#include <math.h>
#include <time.h>
#include <array>
using std::array;
using std::map;
using std::vector;

struct Pose
{
    Vector3 position;
    Vector3 euler = 0.f;
    Pose(const Vector3 position = Vector3(), const Vector3 euler = 0.f)
    {
        this->position = position;
        this->euler = euler;
    }
};

struct PoseStamped
{
    Header header = {};
    Pose pose = {};
    PoseStamped(const Header &header = Header(),
                const Pose &pose = Pose())
    {
        this->header = header;
        this->pose = pose;
    }
};

struct PoseWithCovariance
{
    Pose pose;
    vector<float> covariance;
    PoseWithCovariance(
        const Pose pose = Pose(),
        const vector<float> covariance = vector<float>())
    {
        this->pose = pose;
        this->covariance = covariance;
    }
};

struct PoseArray
{
    Header header;
    vector<Pose> poses;
    PoseArray(const Header &header = Header(),
              const vector<Pose> &poses = vector<Pose>())
    {
        this->header = header;
        this->poses = poses;
    }
};

struct Twist
{
    Vector3 linear;
    Vector3 angular;
    Twist(const Vector3 linear = Vector3(), const Vector3 angular = Vector3())
    {
        this->linear = linear;
        this->angular = angular;
    }
};

struct TwistStamped
{
    Header header = {};
    Twist twist = {};
    TwistStamped(const Header &header = Header(),
                 const Twist twist = Twist())
    {
        this->header = header;
        this->twist = twist;
    }
};

struct TwistWithCovariance
{
    Twist twist;
    vector<float> covariance;
    TwistWithCovariance(
        const Twist &twist = Twist(),
        const vector<float> &covariance = vector<float>())
    {
        this->twist = twist;
        this->covariance = covariance;
    }
};

struct Accel
{
    Vector3 linear;
    Vector3 angular;
    Accel(const Vector3 linear=Vector3(), const Vector3 angular=Vector3())
    {
        this->linear = linear;
        this->angular = angular;
    }
};
struct Transform
{
    Vector3 translation = {};
    float rotation;
    Transform(const Vector3 &translation, const float &rotation)
    {
        this->translation = translation;
        this->rotation = rotation;
    }
};

struct Wrench
{
    float force;
    float steering;
    Wrench(const float &force = 0.f, const float &steering = 0.f)
    {
        this->force = force;
        this->steering = steering;
    }
};

struct Geometry
{
    Pose pose;
    Twist twist;
    Accel accel;
    Wrench wrench;
    Geometry(const Pose &pose = Pose(), const Twist &twist = Twist(), const Accel &accel = Accel(), const Wrench &wrench = Wrench())
    {
        this->pose = pose;
        this->twist = twist;
        this->accel = accel;
        this->wrench = wrench;
    }
};


struct PolarVector {
	float r = 0.f;
	float th = 0.f;
	PolarVector(const float r = 0.f, const float th = 0.f) {
		this->r = r;
		this->th = th;
	}
	void Debug(string name) {
		cout << "    " << name << "--r:" << r << "--th:" << Math::Rad2Deg(th) << endl;
	}
};

struct PolarVelocity{
	float r_d = 0.f;
	float th_d = 0.f;

	PolarVelocity(const float r_d = 0.f, const float th_d = 0.f) {
		this->r_d = r_d;
		this->th_d = th_d;
	}
	void Debug(string name) {
		cout << "    " << name << "--D[r]:" << r_d << "--D[th]:" << Math::Rad2Deg(th_d) << endl;
	}
};


struct VehicleState
{
    //frame stamp
    Header header;
    int gear = 0;
    VehicleState(
        const Header &header = Header(),
        const float &gear = 0)
    {
        this->header = header;
        this->gear = gear;
    }
};

struct GeometricCalc
{
private:
    GeometricCalc();
    ~GeometricCalc();

public:

    static float Geometry2Velocity(Geometry geometry)
    {
        float rad = geometry.pose.euler.z;
        float xdot = geometry.twist.linear.x;
        float ydot = geometry.twist.linear.y;
        float velocity = xdot * cos(rad) + ydot * sin(rad);
        return velocity;
    }
    static float TwistPose2Velocity(Twist twist, Pose pose)
    {
        float rad = pose.euler.z;
        float xdot = twist.linear.x;
        float ydot = twist.linear.y;
        float velocity = xdot * cos(rad) + ydot * sin(rad);
        return velocity;
    }
};
