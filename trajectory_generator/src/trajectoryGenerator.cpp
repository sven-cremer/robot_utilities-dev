/*
 * trajectoryGenerator.cpp
 *
 *  Created on: Jan 5, 2017
 *      Author: sven
 */

#include <trajectory_generator/trajectoryGenerator.h>


TrajectoryGenerator::TrajectoryGenerator()
{
	numPoints = 0;

	int Nx = 3;
	int Ny = 3;
	double dx = 0.07;
	double dy = 0.10;
	geometry_msgs::Point origin;
	origin.z = 0.5;

	initGrid(Nx, Ny, dx, dy, origin);
}

TrajectoryGenerator::~TrajectoryGenerator()
{

}

void TrajectoryGenerator::initGrid( int Nx, int Ny,
									double dx, double dy,
                                    geometry_msgs::Point origin)
{
	grid_.clear();

	char alphabet = 'A';
	numPoints = 0;
	geometry_msgs::Point p = origin;

	for(int iy=0;iy<Ny;iy++)
	{
		p.y += iy*dy;
		for(int ix=0;ix<Nx;ix++)
		{
			p.x += ix*dx;

			grid_[alphabet] = p;

			alphabet++;
			numPoints++;
		}
		p.x = origin.x;
	}
}

std::vector<geometry_msgs::Point> TrajectoryGenerator::str2Vec(std::string s)
{
	std::vector<geometry_msgs::Point> trajectory;

	for ( int i = 0 ; i < s.length(); i++)
	{
		char key = s[i];

		std::map<char,geometry_msgs::Point>::iterator it = grid_.find(key);

		if(it != grid_.end())
		{
			geometry_msgs::Point p = it->second;
			trajectory.push_back(p);
		}
		else
		{
			std::cout<<"Warning: Did not find entry for key "<<key<<"\n";
		}
	}

	return trajectory;
}

void TrajectoryGenerator::printGridMap()
{
	typedef std::map<char,geometry_msgs::Point>::iterator it_type;

	std::cout<<"key"<<": [\t"<<"x"<<"\t"<<"y"<<"\t"<<"z"<<"\t]\n";
	for(it_type iterator = grid_.begin(); iterator != grid_.end(); iterator++)
	{
	    char key = iterator->first;
	    geometry_msgs::Point p = iterator->second;

		std::cout<<key<<": [\t"<<p.x<<"\t"<<p.y<<"\t"<<p.z<<"\t]\n";
	}
	std::cout<<"---\n";
}

void TrajectoryGenerator::printGridLayout()
{
	typedef std::map<char,geometry_msgs::Point>::iterator it_type;

	double y = 0;	// Assume Nx * Ny grid

	for(it_type iterator = grid_.begin(); iterator != grid_.end(); iterator++)
	{
	    char key = iterator->first;
	    geometry_msgs::Point p = iterator->second;

		if(p.y>y)
		{
			y=p.y;
			std::cout<<"\n";
		}
		std::cout<<key<<" ";
	}
	std::cout<<"\n---\n";
}

