// /*copy rigth 2018 Taiki Tanaka Under GPLlv3 License*/

#pragma once
#include "design/TemplateMethod.h"
#include "define/core_define.h"
#include "define/geom_define.h"
#include "define/nav_define.h"
#include "define/waypoint_define.h"
#include "define/ackermann_define.h"
#include <map>
#include <vector>
using std::map;
using std::vector;

class WaypointSearcher : public TemplateMethod
{
	struct Search
	{
		int positionIndex = 0;
		int range = 50;
		int min = 0;
		int max = 0;
	};

	int lane_array_id;

	Search search;
	
	bool isDebug = true;
	Vector3 waypointTarget = {};
	Geometry geometry;
	VehicleState vehicleState;
	AckermannDrive ackermannDrive;
	VehicleLocation vehicleLocation;

	 int SearchNearestLanes(const LaneArray &lanes);
	int SearchNearestWaypoint(const Lane &lane,float &minDistance);
    Vector3 SearchWaypointTarget(const vector<Waypoint> waypoints,Vector3 starePos);
	Vector3 LookAheadModel(Pose pose);
	void ConnectLane();
	LaneArray lane_array;
	Path globalPath = {};

	//===
	void GetState() override;
	void SpinOnce() override;
	void SetState() override;
	void Start() override;
	//===
public:
	WaypointSearcher();
	~WaypointSearcher();
};