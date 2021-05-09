//
// Created by tanaka-car on 2021/05/08.
//

#ifndef CPP_SNIPPET_TRAJECTORY_H
#define CPP_SNIPPET_TRAJECTORY_H

#include <vector>
#include <string>
namespace std_msgs {
	struct Header {
		double time;
		int seq;
		int stamp;
		std::string frame_id;
	};
}
namespace geometry_msgs{
	struct Point{
		double x;
		double y;
		double z;
	};
	struct Orientation{
		double x;
		double y;
		double z;
		double w;
	};

	struct Vector3 {
		double x;
		double y;
		double z;
	};

	struct Pose{
		Point position;
		Orientation orientation;
	};
	struct Twist{
		Vector3 linear;
		Vector3 angular;
	};
	struct Accel{
		Vector3 linear;
		Vector3 angular;
	};
};

namespace autoware_planning_msgs{
	struct TrajectoryPoint {
		geometry_msgs::Pose pose;
		geometry_msgs::Twist twist;
		geometry_msgs::Accel accel;
	};

	struct Trajectory{
		std_msgs::Header header;
		std::vector<TrajectoryPoint> points;
	};
}



#endif //CPP_SNIPPET_TRAJECTORY_H
