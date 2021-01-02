//
// Created by tanakacar on 2020/04/08.
//

#pragma once
#define _USE_MATH_DEFINES
#include "core_define.h"
#include "geom_define.h"
#include "math_define.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <map>
using std::map;
using std::vector;

struct TF {
	static float Grid2Array(int width, int x, int y)
    {
        return width * y + x;
    }

	static Vector2i Position2Grid(const Vector2 position, const Vector2i offset, const float resolution)
	{
		Vector2i grid = {};
		grid.x = offset.x + int(position.x / resolution);
		grid.y = offset.y + int(position.y / resolution);
		return grid;
	}

	static Vector2 Grid2Position(const Vector2i grid, const Vector2i offset, const float resolution)
	{
		Vector2 position;
		position.x = float((grid.x - offset.x) * resolution);
		position.y = float((grid.y - offset.y) * resolution);
		return position;
	}
	/*
	現在の座標系において位置を指定するとその座標系を基準とする回転変換を行う
	in Pose world coordinate Pose
	out Vector3 world coordinate
	*/
	static Vector3 Local2WorldPosition(const Pose pose, const Vector3 &localPosition)
	{
		float w2lRot = pose.euler.z;
		Vector3 worldPoint = pose.position;
		worldPoint.x += localPosition.x * cos(w2lRot) - localPosition.y * sin(w2lRot);
		worldPoint.y += localPosition.x * sin(w2lRot) + localPosition.y * cos(w2lRot);
		worldPoint.z += localPosition.z;
		return worldPoint;
	}
	static Vector3 World2LocalVector(const Pose &selfPose, const Vector3 &otherPosition) {
		float l2wRot = -selfPose.euler.z;
		Vector3 l_ = Vector3::Sub(otherPosition,selfPose.position);
		Vector3 localVector3 = {};
		localVector3.x = l_.x * cos(l2wRot) - l_.y * sin(l2wRot);
		localVector3.y = l_.x * sin(l2wRot) + l_.y * cos(l2wRot);
		localVector3.z = l_.z;
		return localVector3;
	}

	static Vector3 World2LocalTwist(const Vector3 &selfTwist, const Vector3 &otherTwist, const float selfRot) {
		//変更注意
		float l2wRot = -selfRot;
		Vector3 l_ = Vector3::Sub(otherTwist, selfTwist);
		Vector3 localTwist = {};
		localTwist.x = l_.x * cos(l2wRot) - l_.y * sin(l2wRot);
		localTwist.y = l_.x * sin(l2wRot) + l_.y * cos(l2wRot);
		localTwist.z = l_.z;
		return localTwist;
	}

	//local vector
	static PolarVector LocalCartesian2PolarVector(const Vector3 position) {
		float dist = std::hypot(position.y, position.x);
		float dir = std::atan2(position.y, position.x);
		return {dist, dir};
	}

	//local Vector3 local linear velocity->local polar vector
	static PolarVelocity LocalCartesian2PolarVelocity(const Vector3 vec, const Vector3 &velo) {
		float dist = std::pow(std::hypot(vec.y, vec.x), 2);
		float r_d = (vec.x * velo.x + vec.y * velo.y) / dist;
		float th_d = (vec.x * velo.y - velo.x * vec.y) / dist;
		return {r_d, th_d};
	}

	/**
 * @fn World2LocalPolarVector
 * @brief ワールド座標系における自分からみた他車のPolar座標系における値を返す
 * @param selfPose 自分の姿勢
 * @param otherTwist 他車の姿勢
 * @return PolarVector {r,th}
 */
	static PolarVector World2LocalPolarVector(const Pose &selfPose, const Vector3 &otherVector3) {
		Vector3 localVector3 = TF::World2LocalVector(selfPose, otherVector3);
		PolarVector polarVector = TF::LocalCartesian2PolarVector(localVector3);
		return polarVector;
	}

	/**
 * @fn World2LocalPolarVelocity
 * @brief cartesian座標系における速度からlcoal polar座標系における速度への変換
 * @param selfTwist 自車速度
 * @param otherTwist 他車速度
 * @return PolarVelocity型Polarの速度を返す
	 * */
	//world cartesian to local polar
	static PolarVelocity
	World2LocalPolarVelocity(const Pose selfPose, const Vector3 selfTwist, const Vector3 &otherPosition,
							 const Vector3 &otherTwist) {
		Vector3 localVector3 = TF::World2LocalVector(selfPose, otherPosition);
		Vector3 localTwist = TF::World2LocalTwist(selfTwist, otherTwist, selfPose.euler.z);
		PolarVector polarVector = TF::LocalCartesian2PolarVector(localVector3);
		PolarVelocity localPolarVelocity = TF::LocalCartesian2PolarVelocity(localVector3, localTwist);
		return localPolarVelocity;
	}
};