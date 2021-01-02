#pragma once
#define _USE_MATH_DEFINES
#include "core_define.h"
#include "geom_define.h"
#include "waypoint_define.h"
#include <iostream>
#include <vector>
#include <map>
#include <math.h>
#include <time.h>
#include <array>
using std::array;
using std::cout;
using std::endl;
using std::map;
using std::string;
using std::vector;

struct ObjPose
{
    Header header;
    string type;
    int obj_id;
    PoseArray obj;
    ObjPose(const Header &header = Header(),
            const string &type = "",
            const int obj_id = 0,
            const PoseArray &obj = PoseArray())
    {
        this->header = header;
        this->type = type;
        this->obj_id = obj_id;
        this->obj = obj;
    }
};

struct ScanData{
    Header header;
    vector<float> dists;
    ScanData(const Header &header = Header(),
             const  vector<float> &dists = vector<float>()){
                this->header=header;
                this->dists=dists;
    }
    void Debug(){ }  
};

/*s
# This is a message to hold data from an IMU (Inertial Measurement Unit)
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
# If the covariance of the measurement is known, it should be filled in (if all you know is the
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
# data a covariance will have to be assumed or gotten from some other source
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation
# estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each
# covariance matrix, and disregard the associated estimate.
Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z
*/
struct IMU
{
    Header header = {};
    //geometry_msgs/Quaternion orientation -> now at float Z axis
    Vector3 euler;
    //vector<float> orientation_covariance = {vector<float>(9, 0)};
    //Vector3 angular_velocity = {}-> now at float Z axis;
    Vector3 angular_velocity = {};
    //vector<float> angular_velocity_covariance = {vector<float>(9, 0)};
    Vector3 linear_acceleration;
    //vector<float> linear_acceleration_covariance = {vector<float>(9, 0)};
    IMU(const Header &header = Header(),
        const Vector3 euler = Vector3(),
        const Vector3 angular_velocity = Vector3(),
        const Vector3 linear_acceleration = Vector3())
    {
        this->header = header;
        this->angular_velocity = angular_velocity;
        this->linear_acceleration = linear_acceleration;
    }
    void Debug()
    {
        header.Debug();
        linear_acceleration.Debug("imu acc");
    }
};

/*
std_msgs/Header                 header
uint32                          id
string                          label
float32                         score   #Score as defined by the detection, Optional
std_msgs/ColorRGBA              color   # Define this object specific color
bool                            valid   # Defines if this object is valid, or invalid as defined by the filtering
################ 3D BB
string                          space_frame #3D Space coordinate frame of the object, required if pose and dimensions are defines
geometry_msgs/Pose              pose
geometry_msgs/Vector3           dimensions
geometry_msgs/Vector3           variance
geometry_msgs/Twist             velocity
geometry_msgs/Twist             acceleration
sensor_msgs/PointCloud2         pointcloud
geometry_msgs/PolygonStamped    convex_hull
autoware_msgs/LaneArray         candidate_trajectories
bool                            pose_reliable
bool                            velocity_reliable
bool                            acceleration_reliable
############### 2D Rect
string                          image_frame # Image coordinate Frame,        Required if x,y,w,h defined
int32                           x           # X coord in image space(pixel) of the initial point of the Rect
int32                           y           # Y coord in image space(pixel) of the initial point of the Rect
int32                           width       # Width of the Rect in pixels
int32                           height      # Height of the Rect in pixels
float32                         angle       # Angle [0 to 2*PI), allow rotated rects
sensor_msgs/Image               roi_image
############### Indicator information
uint8                          indicator_state # INDICATOR_LEFT = 0, INDICATOR_RIGHT = 1, INDICATOR_BOTH = 2, INDICATOR_NONE = 3
############### Behavior State of the Detected Object
uint8                           behavior_state # FORWARD_STATE = 0, STOPPING_STATE = 1, BRANCH_LEFT_STATE = 2, BRANCH_RIGHT_STATE = 3, YIELDING_STATE = 4, ACCELERATING_STATE = 5, SLOWDOWN_STATE = 6
#
string[]                        user_defined_info
*/

struct DetectedObject
{
    Header header = {};
    uint id = 0;
    string label = "";
    //float score = 0.f;  //   #Score as defined by the detection, Optional
    string space_frame = "world"; //#3D Space coordinate frame of the object, required if pose and dimensions are defines
    Pose pose;
    Vector3 dimensions;
    // Vector3 variance;
    Twist velocity;
    //Twist acceleration;
    //LaneArray candidate_trajectories;
    DetectedObject(
        const Header &header = Header(),
        const uint id = 0,
        const string label = "",
        const Pose pose = Pose(),
        const Vector3 dimensions = Vector3(),
        const Twist velocity = Twist())
    {
        this->header = header;
        this->id = id;
        this->label = label;
        this->pose = pose;
        this->dimensions = dimensions;
        this->velocity = velocity;
    }
};

/*
std_msgs/Header header
DetectedObject[] objects
*/
struct DetectedObjectArray
{
    Header header;
    vector<DetectedObject> objects = {};
    DetectedObjectArray(
        const Header &header = Header(),
        const vector<DetectedObject> objects = vector<DetectedObject>())
    {
        this->header = header;
        this->objects = objects;
    }
};

struct UltraSonic
{
    Header header;
    string name = {};
    float dist = 0.f;
    UltraSonic(
        const Header &header = Header(),
        const string name = "",
        const float dist = 0.f)
    {
        this->header = header;
        this->name = name;
        this->dist = dist;
    }
    void Debug()
    {
        header.Debug();
        cout << "    "  << "dist" << dist << endl;
    }
};
