// /*copy rigth 2018 Taiki Tanaka Under GPLlv3 License*/

#include "PerceptionBase.h"
#include "define/math_define.h"
#include "define/core_define.h"
#include "define/unreal_define.h"
#include "msgs/auto_msgs.h"
#include "msgs/ros_msgs.h"
#include <math.h>
#include <time.h>
#include <vector>


// Sets default values
APerceptionBase::APerceptionBase()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void APerceptionBase::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void APerceptionBase::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	rec->Update();
	

}
