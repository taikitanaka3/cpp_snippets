// /*copy rigth 2018 Taiki Tanaka Under GPLlv3 License*/

#include "SplineTest.h"
#include "define/unreal_define.h"
#include "Components/SplineComponent.h"
#include "Components/SplineMeshComponent.h"
#include "Engine/GameEngine.h"
#include "Engine.h"

// Sets default values
ASplineTest::ASplineTest()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	
	SplinePath = CreateDefaultSubobject<USplineComponent>(FName("Spline"));
	splineForward = ESplineMeshAxis::Type::X;
}

// Called when the game starts or when spawned
void ASplineTest::BeginPlay()
{
	Super::BeginPlay();
	SetupSpline();
}

// Called every frame
void ASplineTest::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void ASplineTest::SetupSpline()
{
	GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Green, "called in setup spline");

	GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Green, "has a static mesh");
	UWorld *World = this->GetWorld();

	splineLength = UnrealUnit::Unreal2ISOScalar(SplinePath->GetSplineLength());
	float isoDiv = 1.0; //m
	int num = splineLength / isoDiv;
	cout << "Len" << splineLength << endl;
	for (int i = 0; i < num; i++)
	{
		float currentUnrealDist = UnrealUnit::ISO2UnrealScalar((float)i * isoDiv);
		cout << "unrealDist" << currentUnrealDist << "num" << num<< endl;
		FVector unrealPoint = SplinePath->GetWorldLocationAtDistanceAlongSpline(currentUnrealDist);
		FRotator unrealRotation = SplinePath->GetRotationAtDistanceAlongSpline(currentUnrealDist, ESplineCoordinateSpace::Type::Local);
		Vector3 isoPoint = UnrealUnit::Unreal2ISOVector(unrealPoint, false);
		Vector3 isoEuler = UnrealUnit::Unreal2ISOEuler(unrealRotation, false);
		isoPoint.Debug("Spline pos");
		isoPoint.Debug("Spline euler");
		waypoint.pose.pose=UnrealUnit::Unreal2ISOPose(unrealPoint,unrealRotation);
		waypoint.dtlane.lw=1.0f;
		waypoint.dtlane.rw=1.0f;
		
		
	}
	waypoint.dtlane.dist = splineLength;

	for (int32 i = 0; i < (SplinePath->GetNumberOfSplinePoints() - 2); i++)
	{
		//USplineMeshComponent *SplineMesh = NewObject<USplineMeshComponent>(this, USplineMeshComponent::StaticClass());
		//SplineMesh->SetStaticMesh(Mesh);
		//SplineMesh->RegisterComponentWithWorld(World);
		//SplineMesh->CreationMethod = EComponentCreationMethod::UserConstructionScript;
		//SplineMesh->SetMobility(EComponentMobility::Movable);
		//SplineMesh->SetForwardAxis(splineForward, true);
		//SplineMesh->AttachToComponent(MySpline, FAttachmentTransformRules::KeepRelativeTransform);
		//FVector SP = MySpline->GetLocationAtSplinePoint(i, ESplineCoordinateSpace::Type::Local);
		//FVector ST = MySpline->GetTangentAtSplinePoint(i, ESplineCoordinateSpace::Type::Local);
		//FVector EP = MySpline->GetLocationAtSplinePoint(i + 1, ESplineCoordinateSpace::Type::Local);
		//FVector ET = MySpline->GetTangentAtSplinePoint(i + 1, ESplineCoordinateSpace::Type::Local);
		//SplineMesh->SetStartAndEnd(SP, ST, EP, ET, true);
		//SplineMesh->SetCollisionEnabled(ECollisionEnabled::Type::QueryAndPhysics);
	}
}