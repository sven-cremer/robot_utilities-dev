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
#include <tf/tf.h>

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

	struct Grid2d{
		geometry_msgs::Point origin;
		int Nx, Ny;
		double dx, dy;
	};

	Grid2d layout;

	//ros::NodeHandle nh;
	//ros::ServiceServer srv_srv_;

	std::map<char,geometry_msgs::Point> grid_;

	int numPoints;


public:

	TrajectoryGenerator();
	~TrajectoryGenerator();

	// Initialize grid
	void initGrid();
	void initGrid( int Nx, int Ny,
                   double dx, double dy,
                   geometry_msgs::Point origin);

	std::vector<geometry_msgs::Pose> str2Vec(std::string s);
	void printGridMap();
	void printGridLayout();

	// Interpolator with constant linear and angular velocity
	void interpolator(const Eigen::Affine3d &x0, const Eigen::Affine3d &x1, int N, std::vector<Eigen::Affine3d> &result);
	void interpolator(const geometry_msgs::Pose &ps, const geometry_msgs::Pose &pf, int N, std::vector<geometry_msgs::Pose> &result);

};

#endif /* TRAJECTORY_GENERATOR_H_ */
