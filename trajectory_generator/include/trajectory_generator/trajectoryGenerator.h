/*
 * trajectoryGenerator.h
 *
 *  Created on: Jan 5, 2017
 *      Author: sven
 */

#ifndef TRAJECTORY_GENERATOR_H_
#define TRAJECTORY_GENERATOR_H_

// ROS
#include <ros/ros.h>

// Messages
#include <geometry_msgs/Pose.h>

/*! \class TrajectoryGenerator trajectoryGenerator.h "include/trajectoryGenerator.h"
 *  \brief For generating Cartesian motion trajectories
 */
class TrajectoryGenerator
{
private:

	/*!
      ROS Handle
	 */
	ros::NodeHandle nh;

	ros::ServiceServer srv_srv_;

	std::map<char,geometry_msgs::Point> grid_;

	int numPoints;

public:

	TrajectoryGenerator();
	~TrajectoryGenerator();

	void initGrid();
	void str2Vec(std::string s);
	void printGrid();




};

#endif /* TRAJECTORY_GENERATOR_H_ */
