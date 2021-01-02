// /*copy rigth 2018 Taiki Tanaka Under GPLlv3 License*/

#include "DrawTest.h"
#include "DrawDebugHelpers.h"
#include <iostream>
using namespace std;
// Sets default values
ADrawTest::ADrawTest()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	// init variables with values
	LocationOne = FVector(0, 0, 600);
	LocationTwo = FVector(0, -600, 600);
	LocationThree = FVector(0, 600, 600);
	LocationFour = FVector(-300, 0, 600);
	LocationFive = FVector(-400, -600, 600);

	MyBox = FBox(FVector(0, 0, 0), FVector(200, 200, 200));
}

// Called when the game starts or when spawned
void ADrawTest::BeginPlay()
{
	Super::BeginPlay();
}

void ADrawTest::PersistentDraw()
{
	DrawDebugPoint(GetWorld(), LocationOne, 200, FColor(52, 220, 239), true, 999);

	DrawDebugSphere(GetWorld(), LocationTwo, 200, 26, FColor(181, 0, 0), true, 999, 0, 2);

	DrawDebugCircle(GetWorld(), CircleMatrix, 200, 50, FColor(0, 104, 167), true, 999, 0, 10);

	DrawDebugCircle(GetWorld(), LocationFour, 200, 50, FColor(0, 0, 0), true, 999, 0, 10);

	DrawDebugSolidBox(GetWorld(), MyBox, FColor(20, 100, 240), MyTransform, true, 999);

	DrawDebugBox(GetWorld(), LocationFive, FVector(100, 100, 100), FColor::Purple, true, 999, 0, 10);

	DrawDebugLine(GetWorld(), LocationTwo, LocationThree, FColor::Emerald, true, 999, 0, 10);

	DrawDebugDirectionalArrow(GetWorld(), FVector(-300, 600, 600), FVector(-300, -600, 600), 120.f, FColor::Magenta, true, 999, 0, 5.f);

	DrawDebugCrosshairs(GetWorld(), FVector(0, 0, 1000), FRotator(0, 0, 0), 500.f, FColor::White, true, 999, 0);
}

void ADrawTest::TempDraw()
{
	DrawDebugPoint(GetWorld(), LocationOne, 200, FColor(52, 220, 239), false, -1.f);

	DrawDebugSphere(GetWorld(), LocationTwo, 200, 26, FColor(181, 0, 0), false, -1.f, 0, 2);

	DrawDebugCircle(GetWorld(), CircleMatrix, 200, 50, FColor(0, 104, 167), false, -1.f, 0, 10);

	DrawDebugCircle(GetWorld(), LocationFour, 200, 50, FColor(0, 0, 0), false, -1.f, 0, 10);

	DrawDebugSolidBox(GetWorld(), MyBox, FColor(20, 100, 240), MyTransform, false, -1.f);

	DrawDebugBox(GetWorld(), LocationFive, FVector(100, 100, 100), FColor::Purple, false, -1.f, 0, 10);

	DrawDebugLine(GetWorld(), LocationTwo, LocationThree, FColor::Emerald, false, -1.f, 0, 10);

	DrawDebugDirectionalArrow(GetWorld(), FVector(-300, 600, 600), FVector(-300, -600, 600), 120.f, FColor::Magenta, false, -1.f, 0, 5.f);

	DrawDebugCrosshairs(GetWorld(), FVector(0, 0, 1000), FRotator(0, 0, 0), 500.f, FColor::White, false, -1.f, 0);
	cout<<"temp"<<endl;
}

// Called every frame
void ADrawTest::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	TempDraw();
}
