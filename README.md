# bioloid_common
This ROS package contains description's files to simulates and visualizes the bioloid humanoid robot. 


## RViz Usage
Standalone (using TF transform from joint state publisher GUI):
```
$ roslaunch bioloid_description rviz.launch standalone:=true
```

With ros control (using TF from robot joint publisher's joint state from ros control):
```
$ roslaunch bioloid_description rviz.launch
```

![system](bioloid_description/assets/images/bioloid_rviz.png)
