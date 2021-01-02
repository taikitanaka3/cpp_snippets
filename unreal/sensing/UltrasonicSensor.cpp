// /*copy rigth 2018 Taiki Tanaka Under GPLlv3 License*/

#include "UltrasonicSensor.h"
#include "RaderSensor.h"
#include "define/math_define.h"
#include "define/unreal_define.h"
#include "define/sensor_define.h"
#include "define/tf_define.h"
#include "msgs/ros_msgs.h"
#include "msgs/auto_msgs.h"
#include "msgs/sensor_msgs.h"
#include <time.h>
#include <math.h>
#include <vector>
#include <iostream>

using std::cout;
using std::endl;

#include "Math/Color.h"
#include "Engine.h"
#include "DrawDebugHelpers.h"

// Sets default values
AUltrasonicSensor::AUltrasonicSensor()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	cout << "create Ultrasonic Sensor" << endl;
}

void AUltrasonicSensor::Start()
{
	this->ultrasonic.header = {0, time(NULL), "ultrasonic", 0.f};
	this->ultrasonic.name = "Front ultrasonic";
}

void AUltrasonicSensor::GetState()
{
	//本当は速さによってレンジを変える
}
void AUltrasonicSensor::SetState()
{
	SensorMsgs::instance()->ultrasonic = this->ultrasonic;
	if (isDebug)
	{
		Print::Start("Ultrasonic");
		ultrasonic.Debug();
		Print::End("Ultrasonic");
	}
}
void AUltrasonicSensor::Scan()
{
	FHitResult OutHit;
	FCollisionQueryParams CollisionParams;

	basePose = UnrealUnit::Unreal2ISOPose(GetActorLocation(), GetActorRotation(), false);
	endPoint = TF::Local2WorldPoint(basePose, sonicRange);
	basePose.position.Debug("Ultrasonic base");
	endPoint.Debug("Ultrasonic end");
	FVector Start = UnrealUnit::ISO2UnrealFVector<Vector3>(basePose.position);
	FVector End = UnrealUnit::ISO2UnrealFVector<Vector3>(endPoint);
	bool hit = GetWorld()->LineTraceSingleByChannel(OutHit, Start, End, ECC_Visibility, CollisionParams);

	if (hit)
	{
		if (OutHit.bBlockingHit)
		{
			cout << "hitしました" << endl;
			Vector3 pos = UnrealUnit::Unreal2ISOVector(OutHit.ImpactPoint);
			pos.Debug("ultrasonic hit pos");
			ultrasonic.dist = UnrealUnit::Unreal2ISOScalar(OutHit.Distance);
			cout << "hit dist:" << ultrasonic.dist << endl;
		}
	}
	else
	{
		ultrasonic.dist = 100.f;
	}
}

void AUltrasonicSensor::SpinOnce()
{
	Scan();
	cout << "scanしました" << endl;
	DrawSensor();
	this->ultrasonic.header.seq++;
}
void AUltrasonicSensor::DrawSensor()
{
	float offsetz = 1.5f;
	UnrealDrawer::DrawLine(GetWorld(), basePose.position, endPoint, FColor::Red, 0.f, 1.f, -1.f);
}

// Called when the game starts or when spawned
void AUltrasonicSensor::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void AUltrasonicSensor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	this->Update();
}
