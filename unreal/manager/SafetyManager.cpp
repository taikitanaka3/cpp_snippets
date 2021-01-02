// /*copy rigth 2018 Taiki Tanaka Under GPLlv3 License*/

#include "SafetyManager.h"

// Sets default values
ASafetyManager::ASafetyManager()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ASafetyManager::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void ASafetyManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	//eStop->Update();
}
