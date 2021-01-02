// /*copy rigth 2018 Taiki Tanaka Under GPLlv3 License*/

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SafetyManager.generated.h"

UCLASS()
class TESTPROJECT_API ASafetyManager : public AActor
{
	GENERATED_BODY()
public:	
	// Sets default values for this actor's properties
	ASafetyManager();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	
	
};
