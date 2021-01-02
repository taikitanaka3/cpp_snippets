// /*copy rigth 2018 Taiki Tanaka Under GPLlv3 License*/

#pragma once
#include "design/TemplateMethod.h"
#include "mission/Passing.h"
#include "mission/Following.h"
#include "mission/Careful.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Mission.generated.h"

//collision smaooth ttc Rule based AI

UCLASS()
class TESTPROJECT_API AMission : public AActor
{
	GENERATED_BODY()
	TemplateMethod *pass = new Passing();
	TemplateMethod *follow=new Following();
	TemplateMethod *careful=new Careful();

public:
	// Sets default values for this actor's properties
	AMission();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};
