/*
 * trajectoryGenerator.h
 *
 *  Created on: Jan 5, 2017
 *      Author: sven
 */

#ifndef TRAJECTORY_GENERATOR_H_
#define TRAJECTORY_GENERATOR_H_

// ROS
//#include <ros/ros.h>

// Math
#include <Eigen/Core>
#include <Eigen/Geometry>

// Messages
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

/*! \class TrajectoryGenerator trajectoryGenerator.h "include/trajectoryGenerator.h"
 *  \brief For generating Cartesian motion trajectories
 */
class TrajectoryGenerator
{
private:

	//ros::NodeHandle nh;
	//ros::ServiceServer srv_srv_;

	std::map<char,geometry_msgs::Point> grid_;

	int numPoints;

public:

	TrajectoryGenerator();
	~TrajectoryGenerator();

	void initGrid( int Nx, int Ny,
                   double dx, double dy,
                   geometry_msgs::Point origin);

	std::vector<geometry_msgs::Pose> str2Vec(std::string s);
	void printGridMap();
	void printGridLayout();


};

#endif /* TRAJECTORY_GENERATOR_H_ */
