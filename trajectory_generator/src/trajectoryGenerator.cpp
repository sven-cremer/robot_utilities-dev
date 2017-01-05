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
}

TrajectoryGenerator::~TrajectoryGenerator()
{

}

void TrajectoryGenerator::initGrid()
{
	numPoints = 0;

	double x_origin = 0.0;
	double y_origin = 0.0;

	char alphabet = 'A';
	geometry_msgs::Point p;

	p.x = x_origin;
	p.y = y_origin;
	p.z = 0.5;

	double dx = 0.07;
	double dy = 0.10;

	int Nx = 3;
	int Ny = 3;

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
		p.x = x_origin;
	}

}

void TrajectoryGenerator::str2Vec(std::string s)
{

	std::vector<geometry_msgs::Point> trajectory;

	for ( int i = 0 ; i < s.length(); i++)
	{
		char key = s[i] ;

		geometry_msgs::Point p = grid_[key];
		trajectory.push_back(p);

		std::cout<<key<<": [\t"<<p.x<<"\t"<<p.y<<"\t"<<p.z<<"\t]\n";
	}

	//return trajectory;

}

void TrajectoryGenerator::printGrid()
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


