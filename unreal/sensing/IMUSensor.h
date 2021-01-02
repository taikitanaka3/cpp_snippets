// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "define/core_define.h"
#include "define/nav_define.h"
#include "define/sensor_define.h"
#include "design/TemplateMethod.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "IMUSensor.generated.h"

UCLASS()
class TESTPROJECT_API AIMUSensor : public AActor, public TemplateMethod
{
	GENERATED_BODY()
	bool isDebug = true;
	bool isActive = false;
	IMU imu;
	Odmetry odm;
	void GetState() override;
	void SpinOnce() override;
	void SetState() override;
	void Start() override;

public:
	// Sets default values for this actor's properties
	AIMUSensor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};
