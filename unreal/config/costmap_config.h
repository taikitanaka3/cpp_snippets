#pragma once

namespace local_costmap
{
const static int limmitIndex = 100;
const static int crashIndex = 99;
const static int warning_index = 97;
const static int minimum_index = 1;
} // namespace local_costmap

namespace gobal_costmap
{
const static int limmitIndex = 100;
const static int crashIndex = 99;
const static int warning_index = 97;
const static int minimum_index = 1;
} // namespace gobal_costmap

namespace occupancy_grid
{
const static float warning_ratio = 0.8f;
const static float dangerous_ratio = 0.4f;
const static float crash_ratio = 0.2f;
//const static float crash_size = 3.0f;
//const static float clearance_size = 4.0f;
} // namespace occupancy_grid
