// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "planning/WaypointSearcher.h"
#include "design/TemplateMethod.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/waypoint_define.h"
#include <map>
#include <vector>
using std::map;
using std::vector;

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GlobalPlanner.generated.h"



UCLASS()
class TESTPROJECT_API AGlobalPlanner : public AActor
{
	GENERATED_BODY()
	//TemplateMethod *gp = new GlobalPlanner();
	TemplateMethod *ws=new WaypointSearcher();
	bool isDebug = true;
	void Update();

public:
	// Sets default values for this actor's properties
	AGlobalPlanner();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};
