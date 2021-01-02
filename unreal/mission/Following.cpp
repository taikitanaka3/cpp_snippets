// /*copy rigth 2018 Taiki Tanaka Under GPLlv3 License*/

#include "Following.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "msgs/sensor_msgs.h"
#include "msgs/ros_msgs.h"
#include "msgs/auto_msgs.h"
#include "config/vehicle_config.h"
#include "config/control_config.h"
#include <iostream>
#include <math.h>
#include <vector>
using std::cout;
using std::endl;
using std::vector;
//=
#include "define/unreal_define.h"
#include "DrawDebugHelpers.h"
#include "Math/Color.h"

Following::Following()
{
    cout << "Following class" << endl;
}

Following::~Following()
{
    cout << "de pasing class" << endl;
}

float Following::Follow(float dist)
{
    float gain = 4.f;
    float speed = gain * (dist - safetyDistance);
    distOld = dist;
    speed = Math::Saturation(speed, -ctrl::follow_speed_range, +ctrl::follow_speed_range);
    return speed;
}
void Following::GetState()
{
    this->ultraSonic = SensorMsgs::instance()->ultrasonic;
    this->accelCmd = VehicleCmdMsgs::instance()->accelCmd;
    this->controlCommand = VehicleCmdMsgs::instance()->controlCommand;
    this->vehicleState = VehicleStateMsgs::instance()->vehicleState;
    this->ackermannDrive = AckermannMsgs::instance()->ackermannDrive;
}
void Following::SpinOnce()
{
    float dist = ultraSonic.dist;

    // float thw=dist/ackermannDrive.speed;
    // float ttc=dist/(ackermannDrive.speed-otherVelo);
    const float StopDistCoeff = 0.5f;
    safetyDistance = vehicle::CAR_LENGTH + safety::STANDSTILL_DISTANCE + StopDistCoeff * ackermannDrive.speed;
    controlCommand.linear_velocity = ctrl::target_speed;
    if (dist < safetyDistance)
    {
        float speed = Follow(dist);
        controlCommand.linear_velocity += speed;
    }
    else
    {
        controlCommand.linear_velocity = ctrl::target_speed;
    }
}
void Following::SetState()
{
    VehicleCmdMsgs::instance()->controlCommand = this->controlCommand;
    if (isDebug)
    {
        Print::Start("Following");
        controlCommand.Debug();
        Print::End("Following");
    }
}