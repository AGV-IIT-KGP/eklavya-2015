Steps to run the planner
========================

* Install `move_base` package  
  `sudo apt-get install ros-${ROS_DISTRO}-move-base`
* Install `robot_pose_ekf` package   
  `sudo apt-get install ros-${ROS_DISTRO}-robot-pose-ekf`  
* Download the gazebo models and extract them and then add the address to the following in `~/.bashrc`  
  `export GAZEBO_MODEL_PATH+=:/path_to_models`  
  `export GAZEBO_RESOURCE_PATH+=:/path_to_models`  
* Install `OMPL` package for your ROS distro  
  `sudo apt-get install ros-${ROS_DISTRO}-ompl`  
