#include <trajectory_planner/circle_trajectory.h>

CircleTrajectory::CircleTrajectory()
{

}

CircleTrajectory::~CircleTrajectory()
{

}

void CircleTrajectory::generate()
{
	//Azumith: polar coordinate running from 0 to pi
	double azumith;
	//Theta: azimuthal coordinate from 0 to 2pi
	double theta;
	/*
	 * Further information for parameterizing a sphere can be found in
	 * http://mathworld.wolfram.com/Sphere.html
	 */
	//Option 1
	double x = std::sqrt(std::pow(radius,2)-(radius*std::cos(azumith)))*std::cos(theta);
	double y = std::sqrt(std::pow(radius,2)-(radius*std::cos(azumith)))*std::sin(theta);
	double z = radius*std::cos(azumith);

	//Option 2
	double x_cart = radius*std::cos(theta)*std::sin(azumith);
	double y_cart = radius*std::sin(theta)*std::sin(azumith);
	double z_cart = radius*std::cos(azumith);


}

int main(int argc, char **argv)
{
  // Init the ROS node
  ros::init(argc, argv, "circular_trajectory_generator");
}
