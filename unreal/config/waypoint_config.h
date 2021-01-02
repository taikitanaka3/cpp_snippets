#pragma once

namespace lane_config
{
enum lane_index
{
	LEFT,
	MIDDLE,
	RIGHT
};
enum lane_direction
{
	// Lane Direction
	FORWARD = 0,
	FORWARD_LEFT = 1,
	FORWARD_RIGHT = 2,
	BACKWARD = 3,
	BACKWARD_LEFT = 4,
	BACKWARD_RIGHT = 5,
	STANDSTILL = 6
};
} // namespace lane_config
namespace lane_change
{
const static float laneChangeDecisionTime = 0.f;
const static float laneChangeStartTime = 5.f;
const static float laneChangeEndTime = 10.f;
} // namespace lane_change

namespace lane_select{
const static float distance_threshold_neighbor_lanes=0.8f;
const static float lane_change_interval=1.5f;
const static float lane_change_target_ratio=0.5f;
const static float lane_change_target_minimum=0.1f;
const static float vector_length_hermite_curve=0.1f;
}


namespace lane_rule{
static float acceleration;
static float stopline_search_radius;
static int number_of_zeros_ahead;
static int number_of_zeros_behind;
static int number_of_smoothing_count;
}