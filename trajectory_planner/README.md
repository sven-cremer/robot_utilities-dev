## Trajectory Planner

This package contains a *CircularTrajectory* class for generating an array of positions that will be used as a trajectory. A configuration file can used to generate different types of the same trajectory and a **ros service call can be used
to recieve the trajectories into a controller class**. 

## Editing the configuration file

First modify the configuration file located in /config/circular_trajectory.yaml. Below is an explanation of the configurating the file correctly. The configuration file contains the following options: 

1. Theta: The maximum angular distance you wish to travel. This parameter must be between 0 and 2pi.
  * Best to leave it as 2*pi.
2. Azumith: The maximum angular distance you wish to travel in 3D when sphere is enabled. This parameter must be between 0 and pi. 
  * Best to leave it as pi. 
3. Radius: Radius of the desired circular trajectory. The generated trajectory maintains a constant radial distance away from the origin. 
4. Resolution: The number of points in the trajectory. Integers only.
5. Sphere: Enables/Disables a 3D trajectory generator. The server will generate a cylindrical trajectory moving about a 3rd axis. 0 to Disable. 1 to Disable. 

## Running
To load the parameters and launch the server, 
```
roslaunch trajectory_planner circular_trajectory.launch 
```