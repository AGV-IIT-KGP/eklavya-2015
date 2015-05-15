Steps to run the planner
========================

* Install `move_base` package  
  `sudo apt-get install ros-indigo-move-base`
* Install `robot_pose_ekf` package   
  `sudo apt-get install ros-indigo-robot-pose-ekf`  
* Download the gazebo models and extract them and then add the address to the following in `~/.bashrc`  
  `export GAZEBO_MODEL_PATH+=:/path_to_models`  
  `export GAZEBO_RESOURCE_PATH+=:/path_to_models`  
