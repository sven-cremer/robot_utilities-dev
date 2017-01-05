/*
 * trajectory_generator_node.cpp
 *
 *  Created on: Jan 5, 2017
 *      Author: sven
 */


#include <trajectory_generator/trajectoryGenerator.h>
/***********************************************************************************************************************
int main(int argc, char **argv)
program entry point
 ***********************************************************************************************************************/

int main(int argc, char **argv)
{

	ros::init(argc, argv, "trajectory_generator_node");
	ros::NodeHandle nh;

	TrajectoryGenerator tg;

	tg.initGrid();

	tg.printGrid();

	std::string s = "ABACH";
	tg.str2Vec(s);

	ROS_INFO("Done!");
	ros::shutdown();
	return 0;

}

