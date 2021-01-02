// Copyright 1998-2019 Epic Games, Inc. All Rights Reserved.

#include "TestProjectPawn.h"
#include "TestProjectWheelFront.h"
#include "TestProjectWheelRear.h"
#include "TestProjectHud.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/InputComponent.h"
#include "WheeledVehicleMovementComponent4W.h"
#include "Engine/SkeletalMesh.h"
#include "Engine/Engine.h"
#include "UObject/ConstructorHelpers.h"
#include "Components/TextRenderComponent.h"
#include "Materials/Material.h"
#include "GameFramework/Controller.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "define/unreal_define.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/ctrl_define.h"
#include "msgs/ros_msgs.h"
#include "msgs/auto_msgs.h"
#include "define/waypoint_define.h"
#include "msgs/auto_msgs.h"
#ifndef HMD_MODULE_INCLUDED
#define HMD_MODULE_INCLUDED 0
#endif

// Needed for VR Headset
#if HMD_MODULE_INCLUDED
#include "IXRTrackingSystem.h"
#include "HeadMountedDisplayFunctionLibrary.h"
#endif // HMD_MODULE_INCLUDED

const FName ATestProjectPawn::LookUpBinding("LookUp");
const FName ATestProjectPawn::LookRightBinding("LookRight");

#define LOCTEXT_NAMESPACE "VehiclePawn"

ATestProjectPawn::ATestProjectPawn()
{
	cout << "create vehicle at test pawn" << endl;
	// Car mesh
	static ConstructorHelpers::FObjectFinder<USkeletalMesh> CarMesh(TEXT("/Game/Vehicle/Sedan/Sedan_SkelMesh.Sedan_SkelMesh"));
	GetMesh()->SetSkeletalMesh(CarMesh.Object);

	static ConstructorHelpers::FClassFinder<UObject> AnimBPClass(TEXT("/Game/Vehicle/Sedan/Sedan_AnimBP"));
	GetMesh()->SetAnimInstanceClass(AnimBPClass.Class);

	// Simulation
	UWheeledVehicleMovementComponent4W *Vehicle4W = CastChecked<UWheeledVehicleMovementComponent4W>(GetVehicleMovement());

	check(Vehicle4W->WheelSetups.Num() == 4);

	Vehicle4W->WheelSetups[0].WheelClass = UTestProjectWheelFront::StaticClass();
	Vehicle4W->WheelSetups[0].BoneName = FName("Wheel_Front_Left");
	Vehicle4W->WheelSetups[0].AdditionalOffset = FVector(0.f, -12.f, 0.f);

	Vehicle4W->WheelSetups[1].WheelClass = UTestProjectWheelFront::StaticClass();
	Vehicle4W->WheelSetups[1].BoneName = FName("Wheel_Front_Right");
	Vehicle4W->WheelSetups[1].AdditionalOffset = FVector(0.f, 12.f, 0.f);

	Vehicle4W->WheelSetups[2].WheelClass = UTestProjectWheelRear::StaticClass();
	Vehicle4W->WheelSetups[2].BoneName = FName("Wheel_Rear_Left");
	Vehicle4W->WheelSetups[2].AdditionalOffset = FVector(0.f, -12.f, 0.f);

	Vehicle4W->WheelSetups[3].WheelClass = UTestProjectWheelRear::StaticClass();
	Vehicle4W->WheelSetups[3].BoneName = FName("Wheel_Rear_Right");
	Vehicle4W->WheelSetups[3].AdditionalOffset = FVector(0.f, 12.f, 0.f);

	// Create a spring arm component
	SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm0"));
	SpringArm->TargetOffset = FVector(0.f, 0.f, 200.f);
	SpringArm->SetRelativeRotation(FRotator(-25.f, 0.f, 0.f));
	SpringArm->SetupAttachment(RootComponent);
	SpringArm->TargetArmLength = 3000.0f;
	SpringArm->bEnableCameraRotationLag = true;
	SpringArm->CameraRotationLagSpeed = 7.f;
	SpringArm->bInheritPitch = false;
	SpringArm->bInheritRoll = false;

	// Create camera component
	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera0"));
	Camera->SetupAttachment(SpringArm, USpringArmComponent::SocketName);
	Camera->bUsePawnControlRotation = false;
	Camera->FieldOfView = 90.f;

	// Create In-Car camera component
	InternalCameraOrigin = FVector(0.0f, -30.0f, 150.0f);

	InternalCameraBase = CreateDefaultSubobject<USceneComponent>(TEXT("InternalCameraBase"));
	InternalCameraBase->SetRelativeLocation(InternalCameraOrigin);
	InternalCameraBase->SetupAttachment(GetMesh());

	InternalCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("InternalCamera"));
	InternalCamera->bUsePawnControlRotation = false;
	InternalCamera->FieldOfView = 90.f;
	InternalCamera->SetupAttachment(InternalCameraBase);

	//Setup TextRenderMaterial
	static ConstructorHelpers::FObjectFinder<UMaterial> TextMaterial(TEXT("Material'/Engine/EngineMaterials/AntiAliasedTextMaterialTranslucent.AntiAliasedTextMaterialTranslucent'"));

	UMaterialInterface *Material = TextMaterial.Object;

	// Create text render component for in car speed display
	InCarSpeed = CreateDefaultSubobject<UTextRenderComponent>(TEXT("IncarSpeed"));
	InCarSpeed->SetTextMaterial(Material);
	InCarSpeed->SetRelativeLocation(FVector(70.0f, -75.0f, 99.0f));
	InCarSpeed->SetRelativeRotation(FRotator(18.0f, 180.0f, 0.0f));
	InCarSpeed->SetupAttachment(GetMesh());
	InCarSpeed->SetRelativeScale3D(FVector(1.0f, 0.4f, 0.4f));

	// Create text render component for in car gear display
	InCarGear = CreateDefaultSubobject<UTextRenderComponent>(TEXT("IncarGear"));
	InCarGear->SetTextMaterial(Material);
	InCarGear->SetRelativeLocation(FVector(66.0f, -9.0f, 95.0f));
	InCarGear->SetRelativeRotation(FRotator(25.0f, 180.0f, 0.0f));
	InCarGear->SetRelativeScale3D(FVector(1.0f, 0.4f, 0.4f));
	InCarGear->SetupAttachment(GetMesh());

	// Colors for the incar gear display. One for normal one for reverse
	GearDisplayReverseColor = FColor(255, 0, 0, 255);
	GearDisplayColor = FColor(255, 255, 255, 255);

	// Colors for the in-car gear display. One for normal one for reverse
	GearDisplayReverseColor = FColor(255, 0, 0, 255);
	GearDisplayColor = FColor(255, 255, 255, 255);

	bInReverseGear = false;
}

void ATestProjectPawn::SetupPlayerInputComponent(class UInputComponent *PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	// set up gameplay key bindings
	check(PlayerInputComponent);

	PlayerInputComponent->BindAxis("MoveForward", this, &ATestProjectPawn::MoveForward);
	PlayerInputComponent->BindAxis("MoveRight", this, &ATestProjectPawn::MoveRight);
	PlayerInputComponent->BindAxis("LookUp");
	PlayerInputComponent->BindAxis("LookRight");

	PlayerInputComponent->BindAction("Handbrake", IE_Pressed, this, &ATestProjectPawn::OnHandbrakePressed);
	PlayerInputComponent->BindAction("Handbrake", IE_Released, this, &ATestProjectPawn::OnHandbrakeReleased);
	PlayerInputComponent->BindAction("SwitchCamera", IE_Pressed, this, &ATestProjectPawn::OnToggleCamera);

	PlayerInputComponent->BindAction("ResetVR", IE_Pressed, this, &ATestProjectPawn::OnResetVR);
}

void ATestProjectPawn::MoveForward(float Val)
{
	GetVehicleMovementComponent()->SetThrottleInput(Val);
}

void ATestProjectPawn::MoveRight(float Val)
{
	GetVehicleMovementComponent()->SetSteeringInput(Val);
}

//この役に立たないブレーキとはなんなのか？？？？ー＞今回はPassで
void ATestProjectPawn::SetBrake(float val)
{
	bool brake=false;
	if(val>0.01f) brake=true;	
	GetVehicleMovementComponent()->SetHandbrakeInput(brake);
}

void ATestProjectPawn::OnHandbrakePressed()
{
	GetVehicleMovementComponent()->SetHandbrakeInput(true);
}

void ATestProjectPawn::OnHandbrakeReleased()
{
	GetVehicleMovementComponent()->SetHandbrakeInput(false);
}

void ATestProjectPawn::OnToggleCamera()
{
	EnableIncarView(!bInCarCameraActive);
}

void ATestProjectPawn::EnableIncarView(const bool bState, const bool bForce)
{
	if ((bState != bInCarCameraActive) || (bForce == true))
	{
		bInCarCameraActive = bState;

		if (bState == true)
		{
			OnResetVR();
			Camera->Deactivate();
			InternalCamera->Activate();
		}
		else
		{
			InternalCamera->Deactivate();
			Camera->Activate();
		}

		InCarSpeed->SetVisibility(bInCarCameraActive);
		InCarGear->SetVisibility(bInCarCameraActive);
	}
}

void ATestProjectPawn::Tick(float Delta)
{
	this->dt = Delta;
	Super::Tick(Delta);

	this->Update();
	// Setup the flag to say we are in reverse gear
	bInReverseGear = GetVehicleMovement()->GetCurrentGear() < 0;

	// Update the strings used in the hud (incar and onscreen)
	UpdateHUDStrings();

	// Set the string in the incar hud
	SetupInCarHUD();

	bool bHMDActive = false;

#if HMD_MODULE_INCLUDED
	if ((GEngine->XRSystem.IsValid() == true) && ((GEngine->XRSystem->IsHeadTrackingAllowed() == true) || (GEngine->IsStereoscopic3D() == true)))
	{
		bHMDActive = true;
	}
#endif // HMD_MODULE_INCLUDED
	if (bHMDActive == false)
	{
		if ((InputComponent) && (bInCarCameraActive == true))
		{
			FRotator HeadRotation = InternalCamera->RelativeRotation;
			HeadRotation.Pitch += InputComponent->GetAxisValue(LookUpBinding);
			HeadRotation.Yaw += InputComponent->GetAxisValue(LookRightBinding);
			InternalCamera->RelativeRotation = HeadRotation;
		}
	}
	//SetBrake(10000.f);
}

void ATestProjectPawn::Update()
{
	GetState();
	SpinOnce();
	SetState();
}

void ATestProjectPawn::FakeLocalize()
{

	Pose pose = UnrealUnit::Unreal2ISOPose(GetActorLocation(), GetActorRotation());
	Vector3 velocity = UnrealUnit::Unreal2ISOVector(GetVelocity());
	Vector3 linear_acceleration=(velocity-fakeOdmetry.twist.twist.linear)/dt;
	fakeOdmetry.pose.pose = pose;
	fakeOdmetry.twist.twist.linear = velocity;
	fakeOdmetry.header.seq++;
	fakeIMU.linear_acceleration=linear_acceleration;
	fakeIMU.header.seq++;
}

void ATestProjectPawn::GetWheelState()
{
	/*Wheel base 2.4m*/
	/*ついにたどり着いたWheelのPropこのくらい解説に載せとけよ、普通につかうやつやん！*/
	float angle[4];
	angle[0] = GetVehicleMovementComponent()->Wheels[0]->GetSteerAngle();
	angle[1] = GetVehicleMovementComponent()->Wheels[1]->GetSteerAngle();
	float isoAngle = UnrealUnit::Unreal2ISORot((angle[0] + angle[1]) * 0.5f);
	float isoOmega = (isoAngle - ackermannDrive.steering_angle) / dt;
	ackermannDrive.steering_angle = isoAngle;
	ackermannDrive.steering_angle_velocity = isoOmega;
	ackermannDrive.speed = UnrealUnit::Unreal2ISOScalar(GetVehicleMovement()->GetForwardSpeed());
	//ackermannDrive.header.seq++;
}

void ATestProjectPawn::GetVehicleState()
{
	//state update
	if (GetVehicleMovement()->GetCurrentGear() < 0)
	{
		vehicleState.gear = Gear::State::back;
	}
	//これを入れるとガタガタになるー＞いまのところLFをGear＊LFとしているため
	//else if (GetVehicleMovement()->GetCurrentGear() == 0)
	//{
	//	VehicleStateMsgs::instance()->vehicleState.gear = Gear::State::nutral;
	//}
	else
	{
		vehicleState.gear = Gear::State::forward;
	}
	vehicleState.header.deltaTime = this->dt;
	vehicleState.header.seq++;
}

void ATestProjectPawn::Start()
{
	vehicleState.header = {0, time(NULL), "vehicle state", 0.f};
	fakeOdmetry.header = {0, time(NULL), "fake localization", 0.f};
	fakeIMU.header= {0, time(NULL), "fake fakeIMU", 0.f};
	//ackermannDirveStamepd.header = {0, time(NULL), "ackermann", 0.f};
}
void ATestProjectPawn::SpinOnce()
{
	FakeLocalize();
	GetWheelState();
	GetVehicleState();
}

void ATestProjectPawn::GetState()
{
}

void ATestProjectPawn::SetState()
{
	FakeLocalizationMsgs::instance()->odmetry = this->fakeOdmetry;
	FakeLocalizationMsgs::instance()->imu=this->fakeIMU;
	VehicleStateMsgs::instance()->vehicleState = this->vehicleState;
	AckermannMsgs::instance()->ackermannDrive = this->ackermannDrive;
	bool isManualControl = false;
	if (!isManualControl)
	{
		MoveForward(VehicleCmdMsgs::instance()->accelCmd.throttle);
		MoveRight(VehicleCmdMsgs::instance()->steerCmd.steering);
	}
	bool isABS = true;
	if (isABS)
	{
		cout<<"BRAKE"<<VehicleCmdMsgs::instance()->brakeCmd.brake<<endl;
		SetBrake(VehicleCmdMsgs::instance()->brakeCmd.brake);
	}
	if (isDebug)
	{
		//ackermannDrive.Debug();
	}
}

void ATestProjectPawn::BeginPlay()
{
	Super::BeginPlay();
	this->Start();
	bool bEnableInCar = false;
#if HMD_MODULE_INCLUDED
	bEnableInCar = UHeadMountedDisplayFunctionLibrary::IsHeadMountedDisplayEnabled();
#endif // HMD_MODULE_INCLUDED
	EnableIncarView(bEnableInCar, true);
}

void ATestProjectPawn::OnResetVR()
{
#if HMD_MODULE_INCLUDED
	if (GEngine->XRSystem.IsValid())
	{
		GEngine->XRSystem->ResetOrientationAndPosition();
		InternalCamera->SetRelativeLocation(InternalCameraOrigin);
		GetController()->SetControlRotation(FRotator());
	}
#endif // HMD_MODULE_INCLUDED
}

void ATestProjectPawn::UpdateHUDStrings()
{
	float KPH = FMath::Abs(GetVehicleMovement()->GetForwardSpeed()) * 0.036f;
	int32 KPH_int = FMath::FloorToInt(KPH);

	// Using FText because this is display text that should be localizable
	SpeedDisplayString = FText::Format(LOCTEXT("SpeedFormat", "{0} km/h"), FText::AsNumber(KPH_int));

	if (bInReverseGear == true)
	{
		GearDisplayString = FText(LOCTEXT("ReverseGear", "R"));
	}
	else
	{
		int32 Gear = GetVehicleMovement()->GetCurrentGear();
		GearDisplayString = (Gear == 0) ? LOCTEXT("N", "N") : FText::AsNumber(Gear);
	}
}

void ATestProjectPawn::SetupInCarHUD()
{
	APlayerController *PlayerController = Cast<APlayerController>(GetController());
	if ((PlayerController != nullptr) && (InCarSpeed != nullptr) && (InCarGear != nullptr))
	{
		// Setup the text render component strings
		InCarSpeed->SetText(SpeedDisplayString);
		InCarGear->SetText(GearDisplayString);

		if (bInReverseGear == false)
		{
			InCarGear->SetTextRenderColor(GearDisplayColor);
		}
		else
		{
			InCarGear->SetTextRenderColor(GearDisplayReverseColor);
		}
	}
}

#undef LOCTEXT_NAMESPACE
