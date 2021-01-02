//
// Created by tanakacar on 2020/04/08.
//

#pragma once
#define _USE_MATH_DEFINES
#include <iostream>
#include <map>
#include <math.h>
#include <random>
#include <vector>
#include <chrono> //for std clock
#include <time.h>
using std::cout;
using std::endl;
using std::map;
using std::string;
using std::vector;
using std::chrono::time_point;

struct Timer
{
	std::chrono::high_resolution_clock::time_point st;

	Timer() { reset(); }

	void reset()
	{
		st = std::chrono::high_resolution_clock::now();
	}

	std::chrono::milliseconds::rep elapsed()
	{
		auto ed = std::chrono::high_resolution_clock::now();
		return std::chrono::duration_cast<std::chrono::milliseconds>(ed - st).count();
	}
};

template <class T>
bool chmax(T &a, const T &b)
{
	if (a < b)
	{
		a = b;
		return 1;
	}
	return 0;
}
template <class T>
bool chmin(T &a, const T &b)
{
	if (b < a)
	{
		a = b;
		return 1;
	}
	return 0;
}

struct Print
{
	static void Start(string name)
	{
		cout << ">>>>" << name << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl;
	}
	static void End(string name)
	{
		cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << name << "<<<<" << endl;
	}
};

namespace Gear
{
	enum State
	{
		forward = 1,
		nutral = 0,
		back = -1
	};
}
template <typename TYPE, std::size_t SIZE>
std::size_t array_length(const TYPE (&array)[SIZE])
{
	return SIZE;
}

struct Vector3
{
	float x = 0.f;
	float y = 0.f;
	float z = 0.f;

	Vector3(float x_ = 0.f, float y_ = 0.f, float z_ = 0.f) { x = x_, y = y_, z = z_; }
	Vector3 operator+(Vector3 v3) { return {x + v3.x, y + v3.y, z + v3.z}; }
	Vector3 operator-(Vector3 v3) { return {x - v3.x, y - v3.y, z - v3.z}; }
	Vector3 operator*(Vector3 v3) { return {x * v3.x, y * v3.y, z * v3.z}; }
	Vector3 operator/(Vector3 v3) { return {x / v3.x, y / v3.y, z / v3.z}; }
	Vector3 operator/(float scalar) { return {x / scalar, y / scalar, z / scalar}; }
	static Vector3 Add(const Vector3 &v, const Vector3 &v_) { return {v.x + v_.x, v.y + v_.y, v.z + v_.z}; }
	static Vector3 Sub(const Vector3 &v, const Vector3 &v_) { return {v.x - v_.x, v.y - v_.y, v.z - v_.z}; }
	static Vector3 Mul(const Vector3 &v, const Vector3 &v_) { return {v.x * v_.x, v.y * v_.y, v.z * v_.z}; }
	static Vector3 Div(const Vector3 &v, const Vector3 &v_) { return {v.x / v_.x, v.y / v_.y, v.z / v_.z}; }
	static Vector3 Mul(const Vector3 &v, const float &f) { return {v.x * f, v.y * f, v.z * f}; }
	static Vector3 Div(const Vector3 &v, const float &f) { return {v.x / f, v.y / f, v.z / f}; }
	static float Distance(const Vector3 &v, const Vector3 &v_)
	{
		Vector3 d = Vector3::Sub(v, v_);
		return sqrt(pow(d.x, 2) + pow(d.y, 2) + pow(d.z, 2));
	}
	static float Distance(const Vector3 &v) { return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2)); }
	static Vector3 Derivative(const Vector3 current, const Vector3 old, const float dt) { return Vector3::Div(Vector3::Sub(current, old), dt); }
	void Debug(string name) { cout << "    " << name << "--x:" << x << "--y:" << y << "--z:" << z << endl; }
};

struct Vector2
{
	float x;
	float y;

	//指定がなければFloat型
	Vector2(float x = 0.f, float y = 0.f)
	{
		this->x = x;
		this->y = y;
	}
	static Vector2 Add(const Vector2 &a, const Vector2 &b) { return {a.x + b.x, a.y + b.y}; }
	static Vector2 Sub(const Vector2 &a, const Vector2 &b) { return {a.x - b.x, a.y - b.y}; }
	float CrossProduct(const Vector2 &a, const Vector2 &b) { return a.x * b.y - a.y * b.x; }
	float DotProduct(const Vector2 &a, const Vector2 &b) { return a.x * b.x - a.y * b.y; }
	static float Distance(float a, float b) { return hypot(a, b); }
	static float Distance(Vector2 a, Vector2 b) { return hypot(a.x - b.x, a.y - b.y); }
	void Debug(string name) { cout << "    " << name << "--x:" << x << "--y:" << y << endl; }
};

struct Vector2i
{
	int x;
	int y;
	//指定がなければFloat型
	Vector2i(int x_ = 0.f, int y_ = 0.f) { x = x_, y = y_; }
	static Vector2i Add(const Vector2i &a, const Vector2i &b) { return {a.x + b.x, a.y + b.y}; }
	static Vector2i Sub(const Vector2i &a, const Vector2i &b) { return {a.x - b.x, a.y - b.y}; }
	int CrossProduct(const Vector2i &a, const Vector2i &b) { return a.x * b.y - a.y * b.x; }
	int DotProduct(const Vector2i &a, const Vector2i &b) { return a.x * b.x - a.y * b.y; }
	static float Distance(int a, int y) { return hypot(a, y); }
	static float Distance(Vector2i a, Vector2i b) { return hypot(a.x - b.x, a.y - b.y); }
	void Debug(string name) { cout << "    " << name << "--x:" << x << "--y:" << y << endl; }
};

struct Rect
{
	float r = 0.f;
	float l = 0.f;
	float f = 0.f;
	float b = 0.f;
	Rect(float r_ = 0.f, float l_ = 0.f, float f_ = 0.f, float b_ = 0.f)
	{
		r = r_;
		l = l_;
		f = f_;
		b = b_;
	}
	void Debug(string name) { cout << "    " << name << "--r:" << r << "--l:" << l << "--f:" << f << "--b:" << b << endl; }
};

/*	uint32 seq
	time_t stamp
	string frame_id
	float deltaTime*/
struct Header
{
	int seq;
	time_t stamp;
	string frame_id;
	float deltaTime;
	Header(
			const int &seq = 0,
			const time_t &stamp = time(NULL),
			const string frame_id = "",
			const float &deltaTime = 0.f)
	{
		this->seq = seq;
		this->stamp = stamp;
		this->frame_id = frame_id;
		this->deltaTime = deltaTime;
	}
	void Debug()
	{
		cout << "/---frame:" << frame_id << "---seq:" << seq << "---stamp:" << stamp << "---/" << endl;
	}
	time_t Time()
	{
		return time(NULL);
	}
};

