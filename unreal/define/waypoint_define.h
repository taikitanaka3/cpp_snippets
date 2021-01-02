
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

struct DTLane {
    float dist = 0.f;
    float dir = 0.f;
    /*
    float apara = 0.f;
    float r = 0.f;
    float slope = 0.f;
    float cant = 0.f;
    */
    float lw = 0.f;
    float rw = 0.f;
};

/*A waypoint for each lane*/
struct Waypoint {
    // global id
    int gid = 0;
    // local id
    int lid = 0;
    PoseStamped pose;
    TwistStamped twist;
    DTLane dtlane;
    int change_flag = 0;
    //WaypointState wpstate;
    uint lane_id = 0;
    //uint left_lane_id = 0;
    //uint right_lane_id = 0;
    uint stop_line_id = 0;
    float cost = 0.f;
    //float time_cost = 0.f;
    uint direction = 0;

    Waypoint(
            const int gid = 0,
            const int lid = 0,
            const PoseStamped pose = PoseStamped(),
            const TwistStamped twist = TwistStamped(),
            const DTLane dtlane = DTLane(),
            uint lane_id = 0,
            uint stop_line_id = 0,
            float cost = 0.f,
            uint direction = 0
    ) {
        this->pose = pose;
        this->twist = twist;
        this->dtlane = dtlane;
        this->lane_id = lane_id;
        this->stop_line_id = stop_line_id;
        this->cost = cost;
        this->direction = direction;
    }
};

struct Lane {
    Header header;
    //ココらへんはいっぺんに初期化できないので
    int increment = 0;
    int lane_id = 0;
    vector<Waypoint> waypoints = {};
    uint lane_index = 0;
    float cost = 0;
    float closest_object_distance = 100.f;
    float closest_object_velocity = 100.f;
    bool is_blocked = false;

    Lane(const Header header = Header(),
         const vector<Waypoint> &waypoints = vector<Waypoint>()) {
        this->header = header;
        this->waypoints = waypoints;
    }
};

struct LaneArray {
    int id = 0;
    //これはもともとlane[]
    map<int, Lane> lanes = {};

    LaneArray(const int id = 0,
              const map<int, Lane> &lanes = map<int, Lane>()) {
        this->id = id;
        this->lanes = lanes;
    }
};


struct VehicleLocation {
    Header header;
    int lane_array_id;
    int waypoint_index;

    VehicleLocation(
            const Header &header = Header(),
            const float &lane_array_id = 0,
            const float &waypoint_index = 2.f) {
        this->header = header;
        this->lane_array_id = lane_array_id;
        this->waypoint_index = waypoint_index;
    }

    void Debug() {
        header.Debug();
        cout << "    lane id : " << lane_array_id << "waypoint id:" << waypoint_index << endl;
    }

};