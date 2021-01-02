// /*copy rigth 2018 Taiki Tanaka Under GPLlv3 License*/

#pragma once
#include "design/TemplateMethod.h"
#include "perception/Recognition.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PerceptionBase.generated.h"



UCLASS()
class TESTPROJECT_API APerceptionBase : public AActor
{
	GENERATED_BODY()
	TemplateMethod *rec=new Recognition();
public:	
	// Sets default values for this actor's properties
	APerceptionBase();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	
	
};
