/* 2018 Taiki Tanaka*/
#pragma once
#include "design/TemplateMethod.h"
#include "planning/DWALocalPlanner.h"
#include "planning/StateLatticeLocalPlanner.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "LocalPlanner.generated.h"


UCLASS()
class TESTPROJECT_API ALocalPlanner : public AActor
{
	GENERATED_BODY()

	bool isDebug = true;
	//TemplateMethod *lp = new LocalPlanner();
	//TemplateMethod *lp = new DWALocalPlanner();
	TemplateMethod *sl=new StateLatticeLocalPlanner();
	ALocalPlanner();
	bool bInitialized = false;
	void Update();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};
