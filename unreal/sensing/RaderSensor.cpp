// /*copy rigth 2018 Taiki Tanaka Under GPLlv3 License*/

#include "RaderSensor.h"
#include "define/math_define.h"
#include "define/unreal_define.h"
#include "define/sensor_define.h"
#include "msgs/ros_msgs.h"
#include "msgs/auto_msgs.h"
#include "msgs/sensor_msgs.h"
#include <time.h>
#include <math.h>
#include <vector>


#include "Math/Color.h"
#include "Engine.h"
#include "DrawDebugHelpers.h"

// Sets default values
ARaderSensor::ARaderSensor()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

void ARaderSensor::SpinOnce()
{
	FHitResult OutHit;
	FVector Start = GetActorLocation();
	FVector forwardVector = GetActorRightVector();
	FVector End = ((forwardVector * 1000.f) + Start);
	FCollisionQueryParams CollisionParams;
	DrawDebugLine(GetWorld(), Start, End, FColor::Green, false, 0.1f, 0, 4);
	bool hit=GetWorld()->LineTraceSingleByChannel(OutHit, Start, End, ECC_Visibility, CollisionParams);
	//cout<<"hit";
	if (hit)
	{
		if (OutHit.bBlockingHit)
		{
			//GEngine->AddOnScreenDebugMessage(-1, 1.f, FColor::Red, FString::Printf(TEXT("You are hitting: %s"), *OutHit.GetActor()->GetName()));
			//GEngine->AddOnScreenDebugMessage(-1, 1.f, FColor::Red, FString::Printf(TEXT("Impact Vector3: %s"), *OutHit.ImpactPoint.ToString()));
			//GEngine->AddOnScreenDebugMessage(-1, 1.f, FColor::Red, FString::Printf(TEXT("Normal Vector3: %s"), *OutHit.ImpactNormal.ToString()));
		}
	}
}

// Called when the game starts or when spawned
void ARaderSensor::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void ARaderSensor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	this->Update();
}
