#ifndef MYX_PLANNER_H_
#define MYX_PLANNER_H_

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class myxplanner
{
private:
    /* data */
public:
    void solve(geometry_msgs::Pose start, geometry_msgs::Pose goal, std::vector<geometry_msgs::Pose> waypoints);
};




#endif