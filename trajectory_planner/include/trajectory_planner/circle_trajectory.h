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
#include <cmath>

class CircleTrajectory
{
private:
	//!ROS Handle
	/*!
      ROS Handle
	 */
	ros::NodeHandle nh;

	std::vector<double> trajectory_positions;
	std::vector<double> trajectory_velocities;

	double radius;
public:
	CircleTrajectory();
	~CircleTrajectory();
	void generate();
};

#endif
