// /*copy rigth 2018 Taiki Tanaka Under GPLlv3 License*/

#pragma once
#include "design/TemplateMethod.h"
#include "define/core_define.h"
#include "define/waypoint_define.h"
#include "Components/SplineMeshComponent.h"

#include <vector>
using std::vector;

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SplineLanePublisher.generated.h"

namespace spline_lane
{
const float DIV = 1.0f;
}

UCLASS()
class TESTPROJECT_API ASplineLanePublisher : public AActor, public TemplateMethod
{

	GENERATED_BODY()
	void GetState() override;
	void SpinOnce() override;
	void SetState() override;
	void Start() override;


	void Clear();
	int state=0;
	bool isWait=true;
	Waypoint waypoint;
	Lane lane;
	int id;
	float dist=0.f;
	float scanRange=1000.f;

public:
	// Sets default values for this actor's properties
	ASplineLanePublisher();
	~ASplineLanePublisher();
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	class USplineComponent *SplinePath;
	UPROPERTY(EditAnywhere)
	TEnumAsByte<ESplineMeshAxis::Type> splineForward;
	//int splineForward;
	UPROPERTY(EditAnywhere)
	USplineComponent *testSpline;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	void Draw();
};
