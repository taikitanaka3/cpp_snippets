// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "design/TemplateMethod.h"
#include "design/StatePattern.h"
#include "define/core_define.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "UnitTestActor.generated.h"

class UnitTest : public TemplateMethod
{
private:
	void GetState() override;
	void SpinOnce() override;
	void SetState() override;
	void Start() override;

public:
	UnitTest() { cout << "create unit test" << endl; };
	~UnitTest(){};
};
UCLASS()
class TESTPROJECT_API AUnitTestActor : public AActor, public TemplateMethod
{
	GENERATED_BODY()
	TemplateMethod *ut = new UnitTest();
	Clock clock;
	void GetState() override{};
	void SpinOnce() override{};
	void SetState() override{};
	void Start() override{};

public:
	// Sets default values for this actor's properties
	AUnitTestActor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};
