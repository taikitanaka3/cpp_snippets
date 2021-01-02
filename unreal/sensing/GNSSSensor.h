// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "../define/core_define.h"
#include "../design/TemplateMethod.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GNSSSensor.generated.h"

UCLASS()
class TESTPROJECT_API AGNSSSensor : public AActor, public TemplateMethod
{
	GENERATED_BODY()
	void GetState() override;
	void SpinOnce() override;
	void SetState() override;
	void Start() override;

	bool isDebug = false;
	void RandomNoise();
	void DrawGNSSPoint(const Vector3 &pos);
	float dt;
	Vector3 position_old;

public:
	// Sets default values for this actor's properties
	AGNSSSensor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};
