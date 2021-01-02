// /*copy rigth 2018 Taiki Tanaka Under GPLlv3 License*/

#include "WaypointSearcher.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/waypoint_define.h"
#include "msgs/auto_msgs.h"
#include "msgs/ros_msgs.h"
#include "config/waypoint_config.h"
#include <time.h>
#include <map>
#include <vector>

#include "define/unreal_define.h"
#include "Math/Color.h"
#include "DrawDebugHelpers.h"

WaypointSearcher::WaypointSearcher() {
    cout << "create Waypoint Searcher \n";
}

WaypointSearcher::~WaypointSearcher() {
    cout << "del Waypoint Searcher \n";
}


// get target waypoint from waypoints
int WaypointSearcher::SearchNearestLanes(const LaneArray &lanes) {
    float currentMinDist = 10000.f;
    int nearestLaneId = 0;
    int nearestWaypointIndex = 0;
    for (const std::pair<int, Lane> lane : lanes.lanes) {
        float tmpMinDist = 10000.f;
        int id = lane.first;
        Lane tmpLane = lane.second;
        int tmpIndex = SearchNearestWaypoint(tmpLane, tmpMinDist);
        bool isLower = chmin<float>(currentMinDist, tmpMinDist);
        if (isLower) {
            cout << "updated lower" << endl;
            nearestLaneId = id;
            nearestWaypointIndex = tmpIndex;
        }
    }
    vehicleLocation.lane_array_id = nearestLaneId;
    vehicleLocation.waypoint_index = nearestWaypointIndex;
    cout << "    min dist:" << currentMinDist << endl;
    return nearestLaneId;
}

// get target waypoint from waypoints
int WaypointSearcher::SearchNearestWaypoint(const Lane &lane, float &minDistance) {
    minDistance = 10000.f;
    int nearestIndex = 0;
    for (int i = 0; i < lane.waypoints.size(); i++) {
        float delx = lane.waypoints[i].pose.pose.position.x - geometry.pose.position.x;
        float dely = lane.waypoints[i].pose.pose.position.y - geometry.pose.position.y;
        Vector2 ps = {delx, dely};
        Vector2 v = {geometry.twist.linear.x, geometry.twist.linear.y};
        float tmpDist=hypot(delx, dely);
        bool isLower = chmin<float>(minDistance, tmpDist);
        if (isLower) {
            nearestIndex = i;
        }
    }
    return nearestIndex;
}

// get stare pos from pos speed angle
Vector3 WaypointSearcher::LookAheadModel(Pose pose) {
    //look forward offset
    float look_ahead = 1.5f;
    float look_speed_gain = 1.5f;
    float look_forward_distance;
    Vector3 starePoint = {};
    look_forward_distance = look_ahead + look_speed_gain * ackermannDrive.speed;
    starePoint.x = pose.position.x + look_forward_distance * cos(pose.euler.z);
    starePoint.y = pose.position.y + look_forward_distance * sin(pose.euler.z);
    return starePoint;
}

// get target waypoint from waypoints
Vector3 WaypointSearcher::SearchWaypointTarget(const vector<Waypoint> waypoints, Vector3 starePoint) {
    float min_distance = 10000.f;
    search.min = search.positionIndex - search.range;
    search.max = search.positionIndex + search.range;
    if (search.min < 0) {
        search.min = 0;
        search.max = waypoints.size();
    }
    if (search.max > waypoints.size()) {
        search.min = 0;
        search.max = waypoints.size();
    }

    for (int i = search.min; i < search.max; i++) {
        double delx = starePoint.x - waypoints[i].pose.pose.position.x;
        double dely = starePoint.y - waypoints[i].pose.pose.position.y;
        double dist = sqrt(pow(delx, 2) + pow(dely, 2));
        bool isLower = chmin<float>(min_distance, dist);
        if (isLower) {
            search.positionIndex = i;
            //std::cout<<"i:"<<i<<std::endl;
        }
    }
    Vector3 target = {
            waypoints[search.positionIndex].pose.pose.position.x,
            waypoints[search.positionIndex].pose.pose.position.y,
            0.f};
    return target;
}

//================================
void WaypointSearcher::Start() {
    globalPath.header = {0, 0, "GlobalPath", 0.f};
    vehicleLocation.header = {0, 0, "Vehicle Location", 0.f};
}

void WaypointSearcher::GetState() {
    this->lane_array = LaneArrayMsgs::instance()->lane_array;
    this->geometry = VehicleStateMsgs::instance()->geometry;
    this->vehicleState = VehicleStateMsgs::instance()->vehicleState;
    this->ackermannDrive = AckermannMsgs::instance()->ackermannDrive;
}

static vector<PoseStamped> Waypoint2PoseStampeds(vector<Waypoint> waypoints) {
    vector<PoseStamped> poses;
    for (Waypoint waypoint : waypoints) {
        poses.emplace_back(waypoint.pose);
    }
    return poses;
}

static vector<PoseStamped> Waypoint2PoseStampeds(vector<Waypoint> waypoints, int begin, int end) {
    vector<PoseStamped> poses;
    for (int i = begin; i < end - 1; i++) {
        poses.emplace_back(waypoints[i].pose);
    }
    return poses;
}

void WaypointSearcher::SetState() {
    globalPath.target = this->waypointTarget;
    //globalPath.poses = Waypoint2PoseStampeds(lane_array.lanes[laneIndex].waypoints, search.min, search.max);
    globalPath.poses = Waypoint2PoseStampeds(lane_array.lanes[lane_array_id].waypoints);
    globalPath.header.seq++;
    vehicleLocation.header.seq++;
    VehicleStateMsgs::instance()->vehicleLocation = vehicleLocation;
    NavigationMsgs::instance()->globalPath = this->globalPath;
    if (isDebug) {
        NavigationMsgs::instance()->globalPath.Debug();
        VehicleStateMsgs::instance()->vehicleLocation.Debug();
    }

}

void WaypointSearcher::SpinOnce() {
    Vector3 starePos = LookAheadModel(geometry.pose);
    lane_array_id = SearchNearestLanes(lane_array);
    waypointTarget = SearchWaypointTarget(lane_array.lanes[lane_array_id].waypoints, starePos);
}
