// /*copy rigth 2018 Taiki Tanaka Under GPLlv3 License*/

#pragma once
#include "design/TemplateMethod.h"
#include "define/core_define.h"
#include "define/nav_define.h"
#include "define/sensor_define.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "UltrasonicSensor.generated.h"

UCLASS()
class TESTPROJECT_API AUltrasonicSensor : public AActor, public TemplateMethod
{
	GENERATED_BODY()
	bool isDebug=true;
	void GetState() override;
	void SpinOnce() override;
	void SetState() override;
	void Start() override;

	void Scan();
	void DrawSensor();
	Pose basePose;
	Vector3 endPoint;
	Vector3 sonicRange={40.f,0.f,0.f};
	UltraSonic ultrasonic;
public:	
	// Sets default values for this actor's properties
	AUltrasonicSensor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	
	
};
