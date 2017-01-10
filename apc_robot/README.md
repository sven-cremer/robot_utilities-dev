# apc_robot
This package contains code developed for the PR2, Baxter, and youBot. There are utlities for controlling grippers, torso, head, base, and arms. The *RobotMoveIt* class is used for planning and kinematics. There is a *PR2manager* class and a *BaxterCommander*, which requires the Rethink Robotics SDK for compilation.

## Setup
Select the robot to be controlled in `cfg/robot.yaml`. To load the settings into the parameter server launch
```
 roslaunch apc_robot robot.launch
```
To use a class in a new package, simply include `apc_robot` as a package dependency
```
catkin_create_pkg newPackageName roscpp apc_robot ...
```
and reference the desired header file.

## Printing PR2 robot state
To print the Cartesian end-effector pose and joint state run
```
rosrun apc_robot joint_states_listener.py
rosrun apc_robot print_state <frame_id>
```
where the frame id is optional (default is `torso_lift_link`). It uses the tf listener so it still works with different controllers such as the mannequin mode:
```
roslaunch pr2_mannequin_mode pr2_mannequin_mode.launch
```
## Baxter demos (deprecated)

Start the apc simulatior and then launch MoveIt:
```
rosrun baxter_interface joint_trajectory_action_server.py
roslaunch baxter_moveit_config demo_baxter.launch
```
To demo the work in progress:
```
rosrun apc_baxter testing_node /joint_states:=/robot/joint_states
```
or
```
roslaunch apc_baxter testing_node.launch
```
Tutorials/Code for testing:
```
roslaunch apc_baxter_moveit kinematic_model
rosrun apc_baxter_moveit movegroup_test
```
### Video demo
```
roslaunch apc_baxter apc_baxter.launch
rosrun apc_baxter test_video_demo /joint_states:=/robot/joint_states
```
