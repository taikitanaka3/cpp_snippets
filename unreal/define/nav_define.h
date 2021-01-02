#pragma once
#define _USE_MATH_DEFINES
#include "core_define.h"
#include "geom_define.h"
#include <iostream>
#include <vector>
#include <map>
#include <math.h>
#include <time.h>
#include <array>
using std::array;
using std::map;
using std::vector;
using std::string;
//for velocity
struct MapMetaData
{
    float map_load_time;
    float resolution;
    int width;
    int height;
    Pose origin;

    MapMetaData(
        const float &map_load_time = 0.f, const float &resolution = 0.f,
        const int &width = 0, const int &height = 0, const Pose &pose = Pose())
    {
        this->map_load_time = map_load_time;
        this->resolution = resolution;
        this->width = width;
        this->height = height;
        this->origin = pose;
    }
};

struct GetPlan
{
    Header header;
    //PoseStamped start = {};
    Pose goal;
    float tolerance = 0.f;
    GetPlan(const Header &header = Header(),
            const Pose &pose = Pose())
    {
        this->header = header;
        this->goal = pose;
    }
    void Debug()
    {
        header.Debug();
        goal.position.Debug("goal pos");
    }
};

struct Odmetry
{
    Header header;
    string child_frame_id;
    PoseWithCovariance pose;
    TwistWithCovariance twist;
    Odmetry(const Header &header = Header(),
            const string &child_frame_id = "",
            const PoseWithCovariance pose = PoseWithCovariance(),
            const TwistWithCovariance twist = TwistWithCovariance())
    {
        this->header = header;
        this->child_frame_id = child_frame_id;
        this->pose = pose;
        this->twist = twist;
    }
    void Debug()
    {
        header.Debug();
        pose.pose.position.Debug("odm pos");
        cout << "covxx" << pose.covariance[0] << "covyy" << pose.covariance[4] << "covthth" << pose.covariance[8] << endl;
    }
};

struct Path
{
    Header header;
    vector<PoseStamped> poses = {};
    Vector3 target = {};
    Path(const Header &header = Header(),
         const vector<PoseStamped> poses = vector<PoseStamped>(),
         const Vector3 target = Vector3())
    {
        this->header = header;
        this->poses = poses;
        this->target = target;
    }
    void Debug()
    {
        header.Debug();
        target.Debug("target");
    }
};

struct OccupancyGrid
{
    Header header;
    MapMetaData info;
    vector<int> Data = {};
    Vector2i offset = {};
    OccupancyGrid(const Header &header = Header(),
                  const MapMetaData &info = MapMetaData(),
                  const vector<int> &Data = vector<int>(),
                  const Vector2i &offset = Vector2i())
    {
        this->header = header;
        this->info = info;
        this->Data = Data;
        this->offset = offset;
    }
    void Debug()
    {
        header.Debug();
        info.origin.position.Debug("map orign");
        offset.Debug("map offset");
    }
};
/*
# Navigation Satellite fix for any Global Navigation Satellite System
#
# Specified using the WGS 84 reference ellipsoid
# header.stamp specifies the ROS time for this measurement (the
#        corresponding satellite time may be reported using the
#        sensor_msgs/TimeReference message).
# header.frame_id is the frame of reference reported by the satellite
#        receiver, usually the location of the antenna.  This is a
#        Euclidean frame relative to the vehicle, not a reference
#        ellipsoid.
Header header
# satellite fix status information
NavSatStatus status
# Latitude [degrees]. Positive is north of equator; negative is south.
float64 latitude
# Longitude [degrees]. Positive is east of prime meridian; negative is west.
float64 longitude
# Altitude [m]. Positive is above the WGS 84 ellipsoid
# (quiet NaN if no altitude is available).
float64 altitude
# Position covariance [m^2] defined relative to a tangential plane
# through the reported position. The components are East, North, and
# Up (ENU), in row-major order.
#
# Beware: this coordinate system exhibits singularities at the poles.
float64[9] position_covariance
# If the covariance of the fix is known, fill it in completely. If the
# GPS receiver provides the variance of each measurement, put them
# along the diagonal. If only Dilution of Precision is available,
# estimate an approximate covariance from that.
uint8 COVARIANCE_TYPE_UNKNOWN = 0
uint8 COVARIANCE_TYPE_APPROXIMATED = 1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
uint8 COVARIANCE_TYPE_KNOWN = 3
uint8 position_covariance_type
*/

//uint8 COVARIANCE_TYPE_UNKNOWN=0
//uint8 COVARIANCE_TYPE_APPROXIMATED=1
//uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
//uint8 COVARIANCE_TYPE_KNOWN=3
//std_msgs/Header header
//sensor_msgs/NavSatStatus status
//float64 latitude
//float64 longitude
//float64 altitude
//float64[9] position_covariance
//uint8 position_covariance_type