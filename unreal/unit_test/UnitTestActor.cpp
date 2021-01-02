// Fill out your copyright notice in the Description page of Project Settings.
#define _USE_MATH_DEFINES
#include "UnitTestActor.h"
#include "define/core_define.h"
#include "define/tf_define.h"
#include <iostream>

void UnitTest::SetState()
{
}

void UnitTest::GetState()
{
}

void UnitTest::SpinOnce()
{
}

void UnitTest::Start()
{
	cout << "一回だけのはずなのに二回以上よばれているよね" << endl;
	cout << "一回だけのはずなのに二回以上よばれているよね" << endl;
	cout << "一回だけのはずなのに二回以上よばれているよね" << endl;
	cout << "一回だけのはずなのに二回以上よばれているよね" << endl;
	cout << "一回だけのはずなのに二回以上よばれているよね" << endl;
}

// Sets default values
AUnitTestActor::AUnitTestActor()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void AUnitTestActor::BeginPlay()
{
	Super::BeginPlay();
	for (int i = 0; i < 24; i++)
	{
		cout << i << ":00" << endl;
		clock.Action(i);
	}
}

// Called every frame
void AUnitTestActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	FVector origin;
	FVector bounds;
	GetActorBounds(false, origin, bounds);
	//UE_LOG(LogTemp,Warning,TEXT("origin (x,y,z):%f,%f,%f"),origin.X,origin.Y,origin.Z);
	//UE_LOG(LogTemp,Warning,TEXT("bounds (x,y,z):%f,%f,%f"),bounds.X,bounds.Y,bounds.Z);
	bool isDebug = false;
	if (isDebug)
	{
		Pose pose = {Vector3(1.f, 2.f, 0.f), Vector3(0.f, 0.f, M_PI_2)};
		Vector3 locPoint = {1.f, 2.f, 0.f};
		cout << "a" << endl;
		Vector3 worldPoint = TF::Local2WorldPoint(pose, locPoint);
		cout << "wx" << worldPoint.x << "wy" << worldPoint.y << endl;
		Vector3 localPoint = TF::World2LocalVector(pose, worldPoint);
		cout << "lx" << localPoint.x << "ly" << localPoint.y << endl;
	}
	ut->Update();
	this->Update();
}
