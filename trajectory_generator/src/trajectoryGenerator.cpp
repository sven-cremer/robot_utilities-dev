/*
 * trajectoryGenerator.cpp
 *
 *  Created on: Jan 5, 2017
 *      Author: sven
 */

#include <trajectory_generator/trajectoryGenerator.h>


TrajectoryGenerator::TrajectoryGenerator()
{

}

TrajectoryGenerator::~TrajectoryGenerator()
{

}

void TrajectoryGenerator::initGrid()
{

	double x_origin = 0.0;
	double y_origin = 0.0;

	char alphabet = 'A';
	geometry_msgs::Point p;

	p.x = x_origin;
	p.y = y_origin;
	p.z = 0.5;

	double dx = 0.05;
	double dy = 0.05;

	int Nx = 3;
	int Ny = 4;

	for(int iy=0;iy<Ny;iy++)
	{
		for(int ix=0;ix<Nx;ix++)
		{
			grid_[alphabet] = p;

			p.x += ix*dx;
			alphabet++;
		}
		p.x = x_origin;
		p.y += iy*dy;
	}

}

void TrajectoryGenerator::printGrid()
{
	typedef std::map<char,geometry_msgs::Point>::iterator it_type;

	for(it_type iterator = grid_.begin(); iterator != grid_.end(); iterator++)
	{
	    char key = iterator->first;
	    geometry_msgs::Point p = iterator->second;

		std::cout<<key<<": ("<<p.x<<","<<p.y<<","<<p.z<<")\n";
	}

}
