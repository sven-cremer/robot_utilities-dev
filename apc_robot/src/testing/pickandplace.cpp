/*
 * pickandplace.cpp
 *
 *  Created on: Feb 16, 2015
 *      Author: sven
 * 
 * Repeatability testing of a pick-and-place task
 * 
 */

#include <apc_baxter/apc_baxter_commander.h>
#include <time.h>
#include "../../include/apc_baxter/apc_robot_moveit.h"

/***********************************************************************************************************************
int main(int argc, char **argv)
program entry point, demonstrates use of BaxterMoveit class
***********************************************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apc_testing_node",ros::init_options::AnonymousName);
//  ros::NodeHandle n;

  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  RobotMoveit baxter_moveit;
  BaxterCommander baxter_commander;
  Gripper baxter_grippers;

  std::string tmp;
  time_t tstart, tend;
  struct timeval s_get, e_get;

  bool motion_complete = true;

  /* Initialize */
  if(!baxter_commander.enableRobot())
  {
	  ROS_ERROR("main: Robot could not be enabled!");
  }
  {
	  ROS_INFO("Enabled robot!");
  }

  sleep(0.1);

  baxter_grippers.open(Gripper::LEFT);
  motion_complete = baxter_moveit.moveToPose(RobotMoveit::RIGHT,"right_close_tuck");

  sleep(0.1);

  int trails = 5;

  double delta_t[trails];

	for(int i=0;i<trails;i++)
	{

		ROS_INFO("Press [enter] to start:");
		std::getline(std::cin, tmp);

		ros::Time start = ros::Time::now();
		tstart = time(0);
		gettimeofday(&s_get, NULL);

		baxter_moveit.moveToPose(RobotMoveit::LEFT,"left_above_place");

		baxter_moveit.moveToPose(RobotMoveit::LEFT,"left_pick_front");

		baxter_moveit.moveToPose(RobotMoveit::LEFT,"left_close_pick");

		baxter_moveit.moveToPose(RobotMoveit::LEFT,"left_pick");

		baxter_grippers.close(Gripper::LEFT);

		baxter_moveit.moveToPose(RobotMoveit::LEFT,"left_close_lift");

		baxter_moveit.moveToPose(RobotMoveit::LEFT,"left_pick_front");

		baxter_moveit.moveToPose(RobotMoveit::LEFT,"left_above_place");

		baxter_moveit.moveToPose(RobotMoveit::LEFT,"left_place");

		baxter_grippers.open(Gripper::LEFT);

		baxter_moveit.moveToPose(RobotMoveit::LEFT,"left_above_place");


		gettimeofday(&e_get, NULL);
		tend = time(0);
		ros::Time stop = ros::Time::now();

		delta_t[i]  = ((e_get.tv_sec * 1e6 + e_get.tv_usec) - (s_get.tv_sec * 1e6 + s_get.tv_usec)) * 1e-3;

		std::cout << "ROS: It took "<< stop.toSec() - start.toSec() <<" second(s).\n";
		std::cout << "SYS: It took "<< difftime(tend, tstart) <<" second(s).\n";
		std::cout << "GET: It took "<< delta_t[i]* 1e-3 <<" second(s).\n";

	}

	std::cout<<"Time per trial:\n";
	for(int i=0;i<trails;i++)
	{
		std::cout<<delta_t[i]* 1e-3 <<"\n";
	}


//  geometry_msgs::Pose l_place;
//  l_place.position.x     = 0.4;
//  l_place.position.y     = 0.8;
//  l_place.position.z     = -0.0;
//  l_place.orientation.x  = 0.678223;
//  l_place.orientation.y  = 0.733508;
//  l_place.orientation.z  = 0.0193943;
//  l_place.orientation.w  = 0.0400478;
//
//  geometry_msgs::Pose l_pick;
//  l_pick.position.x     = 1.0;
//  l_pick.position.y     = 0.0;
//  l_pick.position.z     = 0.2;
//  l_pick.orientation.x  = 0.694173;
//  l_pick.orientation.y  = -0.0300281;
//  l_pick.orientation.z  = 0.718911;
//  l_pick.orientation.w  = 0.0197574;
//
//
//  geometry_msgs::Pose l_place_above;
//  l_place_above = l_place;
//  l_place_above.position.z = 0.2;
//
//  geometry_msgs::Pose l_pick_front;
//  l_pick_front = l_pick;
//  l_pick_front.position.x = 0.7;
//
//  double sleep_time = 5.0;
//
//  baxter_moveit.executeCarteGoal(BaxterMoveit::LEFT, l_place_above);
//  sleep(sleep_time);
//
//  baxter_moveit.executeCarteGoal(BaxterMoveit::LEFT, l_pick_front);
//  sleep(sleep_time);
//
//  baxter_moveit.executeCarteGoal(BaxterMoveit::LEFT, l_pick);
//  sleep(sleep_time);
//
//  baxter_grippers.close(Gripper::LEFT);
//
//  baxter_moveit.executeCarteGoal(BaxterMoveit::LEFT, l_pick_front);
//  sleep(sleep_time);
//
//  baxter_moveit.executeCarteGoal(BaxterMoveit::LEFT, l_place_above);
//  sleep(0.5);
//
//  baxter_moveit.executeCarteGoal(BaxterMoveit::LEFT, l_place);
//  sleep(sleep_time);
//
//  baxter_grippers.open(Gripper::LEFT);
//
//  baxter_moveit.executeCarteGoal(BaxterMoveit::LEFT, l_place_above);
//  sleep(sleep_time);

  ROS_INFO("Done!");
  ros::shutdown();

  return 0;

}




