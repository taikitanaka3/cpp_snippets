// /*copy rigth 2018 Taiki Tanaka Under GPLlv3 License*/

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DrawTest.generated.h"

UCLASS()
class TESTPROJECT_API ADrawTest : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ADrawTest();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	void PersistentDraw();
	void TempDraw();
	// declare location variables
	UPROPERTY(EditAnywhere, Category = "Locations")
	FVector LocationOne;

	UPROPERTY(EditAnywhere, Category = "Locations")
	FVector LocationTwo;

	UPROPERTY(EditAnywhere, Category = "Locations")
	FVector LocationThree;

	UPROPERTY(EditAnywhere, Category = "Locations")
	FVector LocationFour;

	UPROPERTY(EditAnywhere, Category = "Locations")
	FVector LocationFive;

	UPROPERTY(EditAnywhere, Category = "Locations")
	FMatrix CircleMatrix;

	UPROPERTY(EditAnywhere, Category = "Locations")
	FBox MyBox;

	UPROPERTY(EditAnywhere, Category = "Locations")
	FTransform MyTransform;
	
	
};
