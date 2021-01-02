#pragma once
#include <map>
#include <string>
using std::string;

namespace gain
{
static std::map<string, const float> throttle =
    {
        {"P", 10.f},
        {"D", 1.f}};
static std::map<string, const float> steer =
    {
        {"P", 5.f},
        {"D", 1.f}};
} // namespace gain

namespace sync_time
{
//float dt = 0.f;
}

namespace ctrl
{
const static float careful_speed=5.f;
const static float target_speed = 10.0f; //m/s->km/hは3.6倍
const static float follow_speed_range=5.f;
} // namespace ctrl



namespace waypoint_follower
{
static int param_flag;
static float velocity;
static float lookahead_distance;
const static float lookahead_ratio=0.f;
const static float minimum_lookahead_distance=0.5f;
const static float displacement_threshold=0.1f;
const static float relative_angle_threshold=0.5f;
} // namespace waypoint_follower


namespace waypoint_refollower{
static string multi_lane_csv;
static bool replanning_mode;
const static float velocity_max=20.f;   //*3.6km/h
const static float velocity_min=5.f;    //*3.6km/h
const static float accel_limit=10.f;
const static float decel_limit=-10.f;
const static float radius_thresh=0.f;
const static float radius_min=0.f;
static bool resample_mode;
static float resample_interval;
const static int velocity_offset=10.f;
const static int end_point_offset=10.f;
static int braking_distance;
static bool replan_curve_mode;
static bool replan_endpoint_mode;
static bool overwrite_vmax_mode;
static bool realtime_tuning_mode;
}


namespace velocity_set{
const static float stop_distance_obstacle=10.f;
const static float stop_distance_stopline=5.f;
const static float detection_range=30.f;
//const static int threshold_points
//const staic float detection_height_top
//const staic float detection_height_bottom
const static float deceleration_obstacle=3.f;
const static float deceleration_stopline=1.f;
const static float velocity_change_limit=0.5f;
const static float deceleration_range=1.f;
const static float temporal_waypoints_size=1.f;
}

namespace planner_selector{
static int latency_num=2;
static int waypoints_num=20;
static float convergence_num=5;
}