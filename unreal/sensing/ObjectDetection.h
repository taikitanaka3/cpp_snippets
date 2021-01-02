// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "design/TemplateMethod.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/sensor_define.h"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ObjectDetection.generated.h"

UCLASS()
class TESTPROJECT_API AObjectDetection : public AActor, public TemplateMethod
{
	GENERATED_BODY()
	void GetState() override;
	void SpinOnce() override;
	void SetState() override;
	void Start() override;

	TSubclassOf<AActor> findClass;
	TArray<AActor *> actors;
	vector<AActor *> vehicles;
	vector<Vector3> objects;
	FString actorName;
	DetectedObjectArray detectedObjectArray;
	ObjPose ObjPose;
	void FindActors();
	void GetDetails();
	void FindComponents(AActor *actor);
	void GetActorLocationFromAActor(AActor *actor);
	void GetActorStateFromAActor(AActor *actor);
	void SetActorLocationToAActor(AActor *actor, float DeltaTime);
	void DrawBoundingBox();

public:
	// Sets default values for this actor's properties
	AObjectDetection();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};
