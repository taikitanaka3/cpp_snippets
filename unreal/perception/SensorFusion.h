// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#pragma once
#include "../define/core_define.h"
#include "../define/geom_define.h"
#include "../define/nav_define.h"
#include "../design/TemplateMethod.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SensorFusion.generated.h"

class SensorFusion : public TemplateMethod
{
	bool isDebug = true;

	Odmetry fakeOdmetry;
	Odmetry kineOdmetry;

	void GetState() override;
	void SpinOnce() override;
	void SetState() override;
	void Start() override;

public:
	SensorFusion();
	~SensorFusion();
};

UCLASS()
class TESTPROJECT_API ASensorFusion : public AActor
{
	GENERATED_BODY()
	bool isDebug = true;
	TemplateMethod *sf = new SensorFusion();

public:
	// Sets default values for this actor's properties
	ASensorFusion();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};
