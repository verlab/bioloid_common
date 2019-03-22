# bioloid_common
This ROS package contains description's files to simulates and visualizes the bioloid humanoid robot. 


## RViz Usage
Standalone (using TF transform from fake joint state publisher GUI):
```
$ roslaunch bioloid_description rviz.launch standalone:=true
```

With ros control (using TF from robot joint publisher's joint state from ros control):
```
$ roslaunch bioloid_description rviz.launch
```

![system](bioloid_description/assets/images/bioloid_rviz.png)


## Gazebo Usage
You can launch the simulation with:

```
$ roslaunch bioloid_gazebo gazebo.launch
```

PRESS PLAY IN GAZEBO ONLY WHEN EVERYTHING IS LOADED (wait for controllers)

![system](bioloid_gazebo/assets/images/bioloid_gazebo.png)

If you want to launch gazebo headless (without GUI) uses:
```
$ roslaunch bioloid_gazebo gazebo.launch uses_gui:=false
```

In order to move the joints using trajectory controller uses:
```
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

To start the simulation call this service:
```
$ rosservice call /gazebo/unpause_physics "{}" 
```

## Topics
```
$ rostopic list
  /clock
  /rosout
  /rosout_agg
  /tf
  /tf_static
  /typea/footl_contact_sensor_state
  /typea/footr_contact_sensor_state
  /typea/imu
  /typea/joint_states
  /typea/joint_trajectory_controller/command
  /typea/joint_trajectory_controller/follow_joint_trajectory/cancel
  /typea/joint_trajectory_controller/follow_joint_trajectory/feedback
  /typea/joint_trajectory_controller/follow_joint_trajectory/goal
  /typea/joint_trajectory_controller/follow_joint_trajectory/result
  /typea/joint_trajectory_controller/follow_joint_trajectory/status
  /typea/joint_trajectory_controller/state
  /typea/odom
```

