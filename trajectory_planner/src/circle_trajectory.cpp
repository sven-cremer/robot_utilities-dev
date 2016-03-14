#include <trajectory_planner/circle_trajectory.h>


CircleTrajectory::CircleTrajectory()
{
	//Read parameters
	ready = true;
	max_theta=2*3.14;
	if(!nh.getParam("/theta", max_theta))
	{
		ready = false;
		ROS_ERROR("Value not loaded from parameter: theta");
	}
	max_azumith = 3.14;
	if(!nh.getParam("/azumith", max_azumith))
	{
		ready = false;
		ROS_ERROR("Value not loaded from parameter: azumith");
	}
	radius = 0.2;
	if(!nh.getParam("/radius", radius))
	{
		ready = false;
		ROS_ERROR("Value not loaded from parameter: radius");
	}
	resolution = 100;
	if(!nh.getParam("/resolution", resolution))
	{
		ready = false;
		ROS_ERROR("Value not loaded from parameter: resolution");
	}
	sphere = false;
	if(!nh.getParam("sphere", sphere))
	{
		ready = false;
		ROS_ERROR("Value not loaded from parameter: sphere");
	}

	axis = XY;
	//Generate trajectory
	generate();
}

CircleTrajectory::~CircleTrajectory()
{

}

void CircleTrajectory::generate()
{
	/*
	 * Further information for parameterizing a sphere can be found in
	 * http://mathworld.wolfram.com/Sphere.html
	 */

	//trajectory_positions = new geometry_msgs::Pose[resolution+1];
	//trajectory_positions.poses.resize(resolution);
	double x;
	double y;
	double z;

	radius = 0.2;
	resolution = 100;
	max_theta = 2*3.14;

	trajectory_positions.resize(resolution+1);

	for(double i=0;i<=resolution;i++)
	{
		//ROS_INFO("%lf",(i/resolution));
		geometry_msgs::Pose point;
		if(axis == XY)
		{
			ROS_INFO("XY");
			x = radius*std::cos(max_theta*(i/resolution));
			y = radius*std::sin(max_theta*(i/resolution));
			z = 0;
		}else if(axis == XZ)
		{
			ROS_INFO("XZ");
			x = radius*std::sin(max_theta*(i/resolution));
			y = 0;
			z = radius*std::cos(max_theta*(i/resolution));
		}else if(axis == YZ)
		{
			ROS_INFO("YZ");
	        x = 0;
	        y = radius*std::sin(max_theta*(i/resolution));
	        z = radius*std::cos(max_theta*(i/resolution));
		}

		if(sphere)
		{
			if(axis == XY)
			{
				ROS_INFO("Sphere XY");
	            x = x*std::sin(max_azumith*(i/resolution));
	            y = y*std::sin(max_azumith*(i/resolution));
	            z = radius*std::cos(max_azumith*(i/resolution));
			}else if(axis == XZ)
			{
				ROS_INFO("Sphere XZ");
	            x = x*std::sin(max_azumith*(i/resolution));
	            y = radius*std::cos(max_azumith*(i/resolution));
	            z = z*std::sin(max_azumith*(i/resolution));
			}else if(axis == YZ)
			{
				ROS_INFO("Sphere YZ");
	            x = radius*std::cos(max_azumith*(i/resolution));
	            y = y*std::sin(max_azumith*(i/resolution));
	            z = z*std::sin(max_azumith*(i/resolution));
			}
		}
		//ROS_INFO("(%lf,%lf,%lf)",x,y,z);
		point.position.x = x;
		point.position.y = y;
		point.position.z = z;
		trajectory_positions[i] = point;

	}

	//Option 1
	//double x = std::sqrt(std::pow(radius,2)-(radius*std::cos(azumith)))*std::cos(theta);
	//double y = std::sqrt(std::pow(radius,2)-(radius*std::cos(azumith)))*std::sin(theta);
	//double z = radius*std::cos(azumith);

	//Option 2
	//double x_cart = radius*std::cos(theta)*std::sin(azumith);
	//double y_cart = radius*std::sin(theta)*std::sin(azumith);
	//double z_cart = radius*std::cos(azumith);


}

bool CircleTrajectory::call(apc_msgs::TrajectoryGenerate::Request &req,
						apc_msgs::TrajectoryGenerate::Response &res)
{
	/*if(!ready){
		ROS_ERROR("Trajectory not ready. Probably missing parameter");
		return false;
	}*/
	res.trajectory_positions.poses = trajectory_positions;
	/*for(int i=0;i<=resolution;i++)
	{
		//res.trajectory_positions.poses[i] = trajectory_positions[i];
	}*/

	return true;
}

int main(int argc, char **argv)
{
	// Init the ROS node
	ros::init(argc, argv, "circular_trajectory_generator");
	ros::NodeHandle nh;

	CircleTrajectory traj_generator;
	ros::ServiceServer service = nh.advertiseService("get_circular_trajectory", &CircleTrajectory::call,&traj_generator);
	ROS_INFO("Ready");

	ros::spin();
}
