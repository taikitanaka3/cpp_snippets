// /*copy rigth 2018 Taiki Tanaka Under GPLlv3 License*/

#include "Mission.h"

// Sets default values
AMission::AMission()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void AMission::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void AMission::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	Print::Start("Mission");
	//pass->Update();
	follow->Update();
	//careful->Update();
	Print::End("Mission");
}
