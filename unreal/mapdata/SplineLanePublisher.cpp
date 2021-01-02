// /*copy rigth 2018 Taiki Tanaka Under GPLlv3 License*/

#include "SplineLanePublisher.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/waypoint_define.h"
#include "msgs/ros_msgs.h"
#include "msgs/auto_msgs.h"
#include "config/waypoint_config.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <map>
#include <string>
using std::cout;
using std::endl;
using std::map;
using std::string;
using std::vector;

#include "define/unreal_define.h"
#include "DrawDebugHelpers.h"
#include "Math/Color.h"
#include "Components/SplineComponent.h"
#include "Engine/GameEngine.h"
#include "Engine.h"

// Sets default values
ASplineLanePublisher::ASplineLanePublisher()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	cout << "create lane spline" << endl;
	SplinePath = CreateDefaultSubobject<USplineComponent>(FName("Spline"));
	splineForward = ESplineMeshAxis::Type::X;
}
ASplineLanePublisher::~ASplineLanePublisher()
{
	cout << "del spline lane publisher" << endl;
}
// Called when the game starts or when spawned
void ASplineLanePublisher::BeginPlay()
{
	Super::BeginPlay();
}

void ASplineLanePublisher::GetState()
{
	Vector3 vehiclePoint = VehicleStateMsgs::instance()->geometry.pose.position;
	Vector3 splinePoint = UnrealUnit::Unreal2ISOVector(SplinePath->GetWorldLocationAtDistanceAlongSpline(0.f));
	dist = Vector3::Distance(vehiclePoint, splinePoint);
	//車両との距離の計算
	if (state == 0 && dist < scanRange)
	{
		state = 1;
	}
}

void ASplineLanePublisher::SpinOnce()
{
	if (state != 1)
		return;
	lane.header = {1, 0, "lanes[right] array", 0.f};
	float splineLength = UnrealUnit::Unreal2ISOScalar(SplinePath->GetSplineLength());
	int num = (int)(splineLength / spline_lane::DIV);
	cout << "Len" << splineLength << endl;
	for (int s = 0; s < num; s++)
	{
		float currentUnrealDist = UnrealUnit::ISO2UnrealScalar((float)s * spline_lane::DIV);
		//cout << "unrealDist" << currentUnrealDist << "num" << num << endl;
		waypoint.pose.pose = UnrealUnit::Unreal2ISOPose(
			SplinePath->GetWorldLocationAtDistanceAlongSpline(currentUnrealDist),
			SplinePath->GetRotationAtDistanceAlongSpline(currentUnrealDist, ESplineCoordinateSpace::Type::Local),
			true);
		waypoint.dtlane.lw = 1.0f;
		waypoint.dtlane.rw = 1.0f;
		waypoint.lid = s;
		waypoint.pose.pose.position.Debug("way pos");
		waypoint.pose.pose.euler.Debug("way eul");
		waypoint.dtlane.dist = splineLength;
		lane.waypoints.emplace_back(waypoint);
	}
	GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Green, "scan lane:" + GetName());
}
void ASplineLanePublisher::SetState()
{
	if (state == 0)
	{
		cout << "state0" << endl;
		return;
	}
	if (state == 1)
	{
		cout << "state1" << endl;
		LaneArrayMsgs::instance()->lane_array.lanes[id] = lane;
		lane.waypoints.clear();
		state = 2;
	}
	if (state == 2)
	{ //車両との距離が十分遠いと
		Clear();
		cout << "state2" << endl;

		//state=0に戻る
	}
}

void ASplineLanePublisher::Clear()
{
	//lane.waypoints.clear();
}

void ASplineLanePublisher::Start()
{
	FString sName = GetName();
	string name = UnrealUnit::Unreal2CppString(GetName());
	lane.lane_id = std::stoi(name);
	id = lane.lane_id;
}

// Called every frame
void ASplineLanePublisher::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	this->Update();

	if (state == 2)
		Draw();
}

void ASplineLanePublisher::Draw()
{
	UnrealDrawer::DrawLane(
		GetWorld(),
		LaneArrayMsgs::instance()->lane_array.lanes[id],
		FColor::Blue, 0.1f, 0.1f, -1.f);
}