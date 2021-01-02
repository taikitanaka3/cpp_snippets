// /*copy rigth 2018 Taiki Tanaka Under GPLlv3 License*/

#pragma once
#include "define/core_define.h"
#include "define/nav_define.h"
#include "define/sensor_define.h"
#include "design/TemplateMethod.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RaderSensor.generated.h"

UCLASS()
class TESTPROJECT_API ARaderSensor : public AActor, public TemplateMethod
{
	GENERATED_BODY()
	void GetState() override{};
	void SpinOnce() override;
	void SetState() override{};
	void Start() override{};
public:	
	// Sets default values for this actor's properties
	ARaderSensor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	
	
};
