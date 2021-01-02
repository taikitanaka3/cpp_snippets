// /*copy rigth 2018 Taiki Tanaka Under GPLlv3 License*/

#pragma once
#include "define/core_define.h"
#include "define/waypoint_define.h"
#include <vector>

using std::vector;
#include "Components/SplineMeshComponent.h"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SplineTest.generated.h"

UCLASS()
class TESTPROJECT_API ASplineTest : public AActor
{
	GENERATED_BODY()

	void SetupSpline();

	//=========Path
	Waypoint waypoint;
	Lane lane;
	vector<Vector3> positions;
	float splineLength = 0.f;

	float unrealLength = 0.f;

public:
	// Sets default values for this actor's properties
	ASplineTest();
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	class USplineComponent *SplinePath;
	UPROPERTY(EditAnywhere)
	TEnumAsByte<ESplineMeshAxis::Type> splineForward;
	//int splineForward;
	UPROPERTY(EditAnywhere)
	USplineComponent *testSpline;

protected:
	// Called when the game starts or when spawned
	virtual void
	BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};
