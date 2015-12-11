/*
 * movegroup_test.cpp
 *
 *  Created on: Dec 13, 2014
 *      Author: sven
 */


#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "movegroup_test", ros::init_options::AnonymousName);
  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // This connects to a running instance of the move_group node
  moveit::planning_interface::MoveGroup group("right_arm");

  // Specify random target
  group.setRandomTarget();

  // Plan the motion and then move the group to the sampled target
  group.move();

  ros::waitForShutdown();
}


