#pragma once
#define _USE_MATH_DEFINES
#include "define/core_define.h"
#include "define/math_define.h"
#include "define/trajectory_define.h"
#include "define/waypoint_define.h"
#include "define/tf_define.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <string>
using std::cout;
using std::endl;
using std::string;
using std::vector;

#include "Engine/Engine.h"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DrawDebugHelpers.h"
#include "Math/Color.h"

class UnrealUnit
{
private:
	UnrealUnit();
	~UnrealUnit();

public:
	static int UnrealString2Int(FString unrealString)
	{
		string cppString = TCHAR_TO_UTF8(*unrealString);
		int cppInt = std::stoi(cppString);
		return cppInt;
	}

	static string Unreal2CppString(FString unrealString)
	{
		string cppString = TCHAR_TO_UTF8(*unrealString);
		return cppString;
	}
	static float Unreal2ISOScalar(float scalar)
	{
		return scalar * 0.01f;
	}
	static float Unreal2ISOSteer(float steer)
	{
		steer *= -1;
		return steer;
	}
	static float Unreal2ISORot(float angle)
	{
		angle *= -M_PI / 180.f;
		return angle;
	}
	static Vector3 Unreal2ISOVector(const FVector &unrealVal, bool withNoise = true)
	{
		Vector3 isoVal = {unrealVal.X, unrealVal.Y, unrealVal.Z};
		isoVal.x *= 0.01f;
		isoVal.y *= -0.01f;
		isoVal.z *= 0.01f;
		if (withNoise)
		{
			isoVal.x = Math::RandomNoise(isoVal.x, 0.0f, 0.1f);
			isoVal.y = Math::RandomNoise(isoVal.y, 0.0f, 0.1f);
			isoVal.z = Math::RandomNoise(isoVal.z, 0.0f, 0.1f);
		}
		return isoVal;
	}

	static Vector3 Unreal2ISOEuler(const FRotator unrealRotator, bool withNoise = true)
	{
		Vector3 isoEuler = {unrealRotator.Roll, unrealRotator.Pitch, unrealRotator.Yaw};
		isoEuler.x *= -M_PI / 180.f;
		isoEuler.y *= -M_PI / 180.f;
		isoEuler.z *= -M_PI / 180.f;
		if (withNoise)
		{
			isoEuler.x = Math::RandomNoise(isoEuler.x, 0.0f, 0.1f * M_PI / 180.f);
			isoEuler.y = Math::RandomNoise(isoEuler.y, 0.0f, 0.1f * M_PI / 180.f);
			isoEuler.z = Math::RandomNoise(isoEuler.z, 0.0f, M_PI / 180.f);
		}
		return isoEuler;
	}

	static Pose Unreal2ISOPose(const FVector &unrealPosition, const FRotator &unrealRotator, bool withNoise = true)
	{
		Vector3 isoPosition = UnrealUnit::Unreal2ISOVector(unrealPosition, withNoise);
		Vector3 isoEuler=UnrealUnit::Unreal2ISOEuler(unrealRotator, withNoise) ;
		Pose isoPose = {isoPosition, isoEuler};
		return isoPose;
	}

	template <typename T>
	static FVector ISO2UnrealFVector(const T &isoVal, const float offsetz = 0.f)
	{
		FVector unrealPos;
		unrealPos.X = isoVal.x * 100.f;
		unrealPos.Y = isoVal.y * -100.f;
		unrealPos.Z = (isoVal.z + offsetz) * 100.f;
		return unrealPos;
	}
	static float ISO2UnrealScalar(float scalar)
	{
		return scalar * 100.f;
	}

	static float ISO2UnrealAngular(const float &angle)
	{
		float unrealAngle = -angle * 180.f / M_1_PI;
		return unrealAngle;
	}
};

class UnrealDrawer
{
private:
	UnrealDrawer();
	~UnrealDrawer();

public:
	//world point color size time
	static void DrawPoint(UWorld *World, const Vector3 &position, FColor color = FColor::Red, float offsetz = 1.5f, float size = 2.f, float time = -1.f)
	{
		FVector unrealPoint = UnrealUnit::ISO2UnrealFVector(position, offsetz);
		DrawDebugPoint(World, unrealPoint, size, color, false, time);
	}
	static void DrawBox(UWorld *World, const Vector3 position, const Vector3 bounds, FColor color = FColor::Red, float offsetz = 1.5f, float time = -1.f)
	{
		FVector center = UnrealUnit::ISO2UnrealFVector(position, offsetz);
		FVector extent = UnrealUnit::ISO2UnrealFVector(bounds, offsetz);
		DrawDebugBox(World, center, extent, color, false, time, 0, 10.f);
	}

	static void DrawArrow(UWorld *World, const Vector3 &positionFrom, const Vector3 &positionTo, FColor color = FColor::Red, float offsetz = 1.5f, float radius = 0.1f, float time = -1.f)
	{
		FVector pointFrom = UnrealUnit::ISO2UnrealFVector(positionFrom, offsetz);
		FVector pointTo = UnrealUnit::ISO2UnrealFVector(positionTo, offsetz);
		float unrealThickness = UnrealUnit::ISO2UnrealScalar(radius);
		const float RADIUS2SIZE = 50.f;
		float unrealSize = unrealThickness * RADIUS2SIZE;
		DrawDebugDirectionalArrow(World, pointFrom, pointTo, unrealSize, color, false, time, 0, unrealThickness);
	}

	static void DrawLine(UWorld *World, const Vector3 &positionFrom, const Vector3 &positionTo, FColor color = FColor::Red, float offsetz = 1.5f, float unrealThickness = 0.1f, float time = -1.f)
	{
		FVector pointFrom = UnrealUnit::ISO2UnrealFVector(positionFrom, offsetz);
		FVector pointTo = UnrealUnit::ISO2UnrealFVector(positionTo, offsetz);
		float thickness = UnrealUnit::ISO2UnrealScalar(unrealThickness);
		DrawDebugLine(World, pointFrom, pointTo, color, false, time, 0, thickness);
	}
	static void DrawSphere(UWorld *World, const Vector3 &position, FColor color = FColor::Red, float offsetz = 1.5f, float radius = 2.f, float time = -1.f)
	{
		FVector point = UnrealUnit::ISO2UnrealFVector(position, offsetz);
		radius = UnrealUnit::ISO2UnrealScalar(radius);
		DrawDebugSphere(World, point, radius, 10, color, false, time, 0, 2.f);
	}

	static void DrawPoints(UWorld *World, const vector<Vector3> &position, FColor color = FColor::Red, float offsetz = 1.5f, float size = 1.f, float time = -1.f)
	{
		for (int i = 0; i < position.size(); i++)
		{
			UnrealDrawer::DrawPoint(World, position[i], color, offsetz, size, time);
		}
	}
	static void DrawCircle(UWorld *World, const Vector3 &position, FColor color = FColor::Red, float offsetz = 1.5f, float radius = 2.f, float time = -1.f)
	{
		FVector point = UnrealUnit::ISO2UnrealFVector(position, offsetz);
		radius = UnrealUnit::ISO2UnrealScalar(radius);
		
		//DrawDebugCircle(world,point,radius,10);
		//DrawDebugSphere(World, point, radius, 10, color, false, time, 0, 2.f);
	}

	static void DrawPath(UWorld *World, const Path &path, FColor color = FColor::Green, float offsetz = 1.6f, float radius = 0.2f, float time = -1.f)
	{
		for (int i = 0; i < path.poses.size() - 1; i++)
		{
			Vector3 start = path.poses[i].pose.position;
			Vector3 end = path.poses[i + 1].pose.position;
			UnrealDrawer::DrawArrow(World, start, end, color, offsetz, radius, time);
		}
		UnrealDrawer::DrawPoint(World, path.target, color, offsetz, radius * 4.f, time);
	}
	static void DrawLane(UWorld *World, const Lane &lane, FColor color = FColor::Red, float offsetz = 1.5f, float radius = 0.2f, float time = -1.f)
	{
		for (int i = 0; i < lane.waypoints.size() - 1; i++)
		{
			Vector3 start = lane.waypoints[i].pose.pose.position;
			Vector3 end = lane.waypoints[i + 1].pose.pose.position;
			UnrealDrawer::DrawArrow(World, start, end, color, offsetz, radius, time);
		}
	}
	static void DrawLoci(UWorld *World, const Loci &drawLoci, FColor color = FColor::Red, float offsetz = 1.7f, float size = 5.f, float time = -1.f)
	{
		cout << "drawLoci.loci.size()" << drawLoci.loci.size() << endl;
		cout << "drawLoci.loci[0].states.size()" << drawLoci.loci[0].states.size() << endl;

		//Locusの数だけfor loopが回る
		for (int i = 0; i < 1; i++) //drawLoci.loci.size(); i++)
		{
			//Locusの点数だけforloopが回る
			for (int j = 20; j < 30; j++) //drawLoci.loci[i].states.size(); j++)
			{
				Vector3 pos = {
					drawLoci.loci[i].states[j].x,
					drawLoci.loci[i].states[j].y,
					0.f};
				UnrealDrawer::DrawPoint(World, pos, color, offsetz, size, time);
				//cout << "i:" << i << "j:" << j << "draw point x:" << point.x << "y:" << point.y << "z:" << point.z << endl;
				//cout << "i:" << i << "j:" << j << "draw locus x:" << tmpP.x << "y:" << tmpP.y << "z:" << tmpP.z << endl;
			}
		}
	}
	static void DrawLocalGrid(UWorld *World, OccupancyGrid grid)
	{
		const float offsetz = 1.7f;
		for (int y = 0; y < grid.info.height; y++)
		{
			for (int x = 0; x < grid.info.width; x++)
			{
				int index = TF::Grid2Array(grid.info.width, x, y);
				int indexVal = grid.Data[index];
				float indexHeight = indexVal * 0.01f + offsetz;
				Vector2i gridPos = {x, y};
				//local座標系における位置が求まった
				Vector2 tmp = TF::Grid2Position(gridPos, grid.offset, grid.info.resolution);
				Vector3 localPoint = {tmp.x, tmp.y, 0.f};
				Vector3 worldPoint = TF::Local2WorldPoint(grid.info.origin, localPoint);
				Vector3 pos = {worldPoint.x, worldPoint.y, indexHeight};
				FColor pointColor = UnrealDrawer::Hight2Color(indexVal);
				UnrealDrawer::DrawPoint(World, pos, pointColor, offsetz, 2.f, -1.f);
			}
		}
	}

	static void DrawWorldGrid(UWorld *World, OccupancyGrid occ)
	{
		const float offsetz = 1.3f;
		for (int y = 0; y < occ.info.height; y++)
		{
			for (int x = 0; x < occ.info.width; x++)
			{
				int index = TF::Grid2Array(occ.info.width, x, y);
				Vector2i gridPos = {x, y};
				Vector2 tmp = TF::Grid2Position(gridPos, occ.offset, occ.info.resolution);
				int indexVal = occ.Data[index];
				float indexHeight = indexVal * 0.01f + offsetz;
				Vector3 pos = {tmp.x, tmp.y, indexHeight};
				FColor pointColor = UnrealDrawer::Hight2Color(indexVal);
				UnrealDrawer::DrawPoint(World, pos, pointColor, offsetz, 2.f, 1000.f);
			}
		}
	}

	static void DrawBoundingBoxes(UWorld *World, const DetectedObjectArray &detectedObject, FColor color = FColor::Red, float offsetz = 1.5f, float time = -1.f)
	{
		for (int i = 0; i < detectedObject.objects.size(); i++)
		{
			UnrealDrawer::DrawBox(World, detectedObject.objects[i].pose.position, detectedObject.objects[i].dimensions, color, offsetz, time);
		}
	}

	static FColor Hight2Color(int indexVal)
	{
		FColor color = FColor::White;
		if (indexVal == 100)
		{
			color = FColor::Yellow;
		}
		else if (50 < indexVal && indexVal <= 99)
		{
			color = FColor::Cyan;
		}
		else if (1 <= indexVal && indexVal <= 50)
		{
			color = FColor::Blue;
		}
		else
		{
			color = FColor::White;
		}
		return color;
	}
};