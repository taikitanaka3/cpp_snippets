// Fill out your copyright notice in the Description page of Project Settings.

#include "ObjectDetection.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/math_define.h"
#include "define/unreal_define.h"
#include "msgs/sensor_msgs.h"
#include "msgs/ros_msgs.h"
#include "msgs/auto_msgs.h"

#include <iostream>
#include <stdio.h>
#include <map>
#include <time.h>
using std::cout;
using std::endl;

#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "Engine/StaticMeshActor.h"
#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"

// Sets default values
AObjectDetection::AObjectDetection()
{
	cout << "create object detection" << endl;
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void AObjectDetection::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void AObjectDetection::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	this->Update(); //ダミーセンサの値取得
}

void AObjectDetection::Start()
{
	FindActors();
	detectedObjectArray.header= {0, time(NULL), "doa", 0.f};
}
void AObjectDetection::GetState()
{
}
void AObjectDetection::SpinOnce()
{
	GetDetails();
	DrawBoundingBox();
}

// Called every frame
void AObjectDetection::SetState()
{
	detectedObjectArray.header.seq++;
	SensorMsgs::instance()->detectedObjectArray = this->detectedObjectArray;
	this->detectedObjectArray.objects.clear();
	
}

void AObjectDetection::DrawBoundingBox(){
	UnrealDrawer::DrawBoundingBoxes(
		GetWorld(),
		detectedObjectArray,
		FColor::Purple,
		1.5f,
		-1.f
	);
}

void AObjectDetection::GetDetails()
{
	if (actors.Num())
	{
		for (int32 i = 0; i < actors.Num(); i++)
		{
			AActor *actor = Cast<AActor>(actors[i]);
			actorName = actor->GetName();
			if (actorName.Contains(TEXT("Objects"), ESearchCase::CaseSensitive, ESearchDir::FromEnd))
			{
				//FindComponents(actor);
				this->GetActorStateFromAActor(actor);
			}
			//delete actor;
		}
	}
}
void AObjectDetection::FindActors()
{
	findClass = AActor::StaticClass();
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), findClass, actors);
}

void AObjectDetection::GetActorStateFromAActor(AActor *actor)
{
	FVector origin;
	FVector bounds;
	actor->GetActorBounds(false,origin,bounds);
	//UE_LOG(LogTemp,Warning,TEXT("origin (x,y,z):<%.4f,%.4f,%.4f>"),origin.X,origin.Y,origin.Z);
	//UE_LOG(LogTemp,Warning,TEXT("bounds (x,y,z):%f,%f,%f"),bounds.X,bounds.Y,bounds.Z);
	string name=UnrealUnit::Unreal2CppString(actor->GetName());
	Pose pose=UnrealUnit::Unreal2ISOPose(actor->GetActorLocation(),actor->GetActorRotation());
	
	Vector3 dimension=UnrealUnit::Unreal2ISOVector(bounds);
	Vector3 velo = UnrealUnit::Unreal2ISOVector(actor->GetVelocity(),false);

	DetectedObject detectedObject;
	detectedObject.pose=pose;
	detectedObject.id=0;
	detectedObject.label=name;
	detectedObject.dimensions=dimension;

	//ObjPose objPose;
	//objPose.obj.poses.emplace_back(pose);
	//objPose.type="person";

	detectedObjectArray.objects.emplace_back(detectedObject);
	//objects.emplace_back(loc);
}

void AObjectDetection::FindComponents(AActor *actor)
{
	TArray<UStaticMeshComponent *> comps;
	actor->GetComponents(comps);
	FString f = actor->GetName();
	for (auto StaticMeshComponent : comps)
	{
		//UE_LOG(LogTemp, Warning, TEXT("Component"));
		FVector ComponentLocation = StaticMeshComponent->GetComponentLocation();
		FName ComponentName_tmp = StaticMeshComponent->GetFName();
		FString ComponentName = ComponentName_tmp.ToString();

		UE_LOG(LogTemp, Warning, TEXT("Component %s"), *ComponentName);

		if (ComponentName.Contains(TEXT("StaticMeshComponent_0"), ESearchCase::CaseSensitive, ESearchDir::FromEnd))
		{
			UE_LOG(LogTemp, Error, TEXT("Obj[%s:%s] <%.4f, %.4f, %.4f>"),
				   *f, *ComponentName, ComponentLocation.X, ComponentLocation.Y, ComponentLocation.Z);
		}
	}
}
