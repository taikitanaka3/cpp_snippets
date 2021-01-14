#include <iostream>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <example/example.h>
#include <example/motion_model.h>

Vehicle::Vehicle() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
    //x_new_ << 1, 1, 1, 1, 1, 1;
    //x_old_ << 1, 1, 1, 1, 1, 1;
    A_ << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 1, 0,
        0, 0, 1, 2, 3, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;
    X_ << 1, 2, 3, 3, 4, 5;
    example_timer_ = pnh_.createTimer(ros::Duration(0.1), &Vehicle::onTimer, this);
}

void Vehicle::onTimer(const ros::TimerEvent &e)
{
    predictKinematicsModel();
}
void Vehicle::predictKinematicsModel()
{
    std::cout << A_ << std::endl;
    std::cout << A_ * X_ << std::endl;
    X_ += A_ * X_ * dt_;
    //ROS_INFO_STREAM();
    //x_new_ = (A_ * x_old_.transpose());
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "motion_model");
    Vehicle node;
    ros::spin();
    return 0;
}
