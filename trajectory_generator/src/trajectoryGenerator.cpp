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

std::vector<geometry_msgs::Pose> TrajectoryGenerator::str2Vec(std::string s)
{
	std::vector<geometry_msgs::Pose> trajectory;

	for ( int i = 0 ; i < s.length(); i++)
	{
		char key = s[i];

		std::map<char,geometry_msgs::Point>::iterator it = grid_.find(key);

		if(it != grid_.end())
		{
			geometry_msgs::Pose p;
			p.position = it->second;	// Set position (geometry_msgs::Point)
			p.orientation.w = 1;		// Set orientation (geometry_msgs::Quaternion)
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

void TrajectoryGenerator::interpolator(const Eigen::Affine3d &x0, const Eigen::Affine3d &x1, int N, std::vector<Eigen::Affine3d> &result)
{
	result.clear();

    Eigen::Vector3d p0(x0.translation());
    Eigen::Vector3d p1(x1.translation());
    Eigen::Vector3d pd = p1 - p0;

	Eigen::Quaterniond q0(x0.linear());
	Eigen::Quaterniond q1(x1.linear());
	q0.normalize();
	q1.normalize();

	tf::Quaternion tf_q0(q0.x(), q0.y(), q0.z(), q0.w());
	tf::Quaternion tf_q1(q1.x(), q1.y(), q1.z(), q1.w());

	double dx = pd.norm() / (N-1);
	double dq = 1.0 / (N-1);

	for(int i=0; i < N; i++)
	{
		tf::Quaternion tf_q = tf_q0.slerp(tf_q1, i*dq);
		Eigen::Quaterniond qi(tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z());

		Eigen::Vector3d pi = p0 + i* dq * pd;

		Eigen::Affine3d xi = Eigen::Translation3d(pi) * qi;
		result.push_back(xi);
	}

}

void TrajectoryGenerator::interpolator(const geometry_msgs::Pose &ps, const geometry_msgs::Pose &pf, int N, std::vector<geometry_msgs::Pose> &result)
{
	// Convert from Pose to Affine3d
	Eigen::Vector3d p0(ps.position.x,ps.position.y,ps.position.z);
	Eigen::Quaterniond q0(ps.orientation.w,ps.orientation.x,ps.orientation.y,ps.orientation.z);
	Eigen::Affine3d x0 = Eigen::Translation3d(p0) * q0;

	Eigen::Vector3d p1(pf.position.x,pf.position.y,pf.position.z);
	Eigen::Quaterniond q1(pf.orientation.w,pf.orientation.x,pf.orientation.y,pf.orientation.z);
	Eigen::Affine3d x1 = Eigen::Translation3d(p1) * q1;

	// Interpolate
	std::vector<Eigen::Affine3d> x_vec;
	interpolator(x0, x1, N, x_vec);

	// Convert Affine3d back to Pose
	result.clear();
	for(int i=0;i<N;i++)
	{
		Eigen::Affine3d x = x_vec[i];
		Eigen::Vector3d p(x.translation());
		Eigen::Quaterniond q(x.linear());

		geometry_msgs::Pose tmp;
		tmp.position.x = p.x();
		tmp.position.y = p.y();
		tmp.position.z = p.z();
		tmp.orientation.x = q.x();
		tmp.orientation.y = q.y();
		tmp.orientation.z = q.z();
		tmp.orientation.w = q.w();

		result.push_back(tmp);
	}
}


