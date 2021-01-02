// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "design/TemplateMethod.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/ackermann_define.h"
#include <vector>
using std::vector;

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Odmetry.generated.h"

namespace odmetry_sorce
{
const int STAMPED_LENGTH = 6;
const int COVARIANCE_DIMENSION = 3;
const int RESET_FREQUENCY = 50;
} // namespace odmetry_sorce

class OdmetrySource : public TemplateMethod
{

private:
	bool isDebug = true;
	Odmetry odmetry;
	AckermannDrive ackermannDrive = {};
	void Estimate(Pose &pose, Twist &twist);
	vector<float> SetCovarianceMatrix(vector<float> data);
	//この形式でないとC++１１以降は怒られる
	vector<vector<float>> xt = {
		odmetry_sorce::COVARIANCE_DIMENSION,
		vector<float>(odmetry_sorce::STAMPED_LENGTH, 0)};

	//===
	void GetState() override;
	void SpinOnce() override;
	void SetState() override;
	void Start() override;

public:
	OdmetrySource();
	~OdmetrySource();
};

UCLASS()
class TESTPROJECT_API AOdmetry : public AActor
{
	GENERATED_BODY()
	TemplateMethod *odm = new OdmetrySource();
	void Initialize();

public:
	// Sets default values for this actor's properties
	AOdmetry();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};
