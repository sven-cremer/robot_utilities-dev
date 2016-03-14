/*!
 * 	\name 	   CircleTrajectory
 *  \brief     Generates a circular trajectory
 *  \details   Generates a set of position and velocity vectors that represent a circular trajectory
 *  \author    Rommel Alonzo
 *  \version   1.0
 *  \date      January 22, 2016
 *  \pre       First initialize the system.
 *  \warning   Improper use can crash your application
 *  \copyright BSD License
 */

#ifndef CIRCULAR_TRAJECTORY_H_
#define CIRCULAR_TRAJECTORY_H_

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <apc_msgs/TrajectoryGenerate.h>
#include <cmath>

class CircleTrajectory
{
//This public section is for enum declaration only
public:
	enum AXIS{
	XY,
	XZ,
	YZ
};
private:
	//!ROS Handle
	/*!
      ROS Handle
	 */
	ros::NodeHandle nh;

	std::vector<geometry_msgs::Pose> trajectory_positions;
	//geometry_msgs::Pose[] trajectory_positions;
	//geometry_msgs::PoseArray trajectory_positions;
	std::vector<double> trajectory_velocities;

	double 	radius;
	double 	max_theta;
	double 	max_azumith;
	int 	resolution;
	int	    sphere;
	AXIS	axis;

	bool	ready;

public:

	CircleTrajectory();
	~CircleTrajectory();
	void generate();
	bool call(apc_msgs::TrajectoryGenerate::Request &req,apc_msgs::TrajectoryGenerate::Response &res);
};

#endif
