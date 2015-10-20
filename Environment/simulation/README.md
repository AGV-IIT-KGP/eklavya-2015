Installing Gazebo 4
===================
Execute out the following commands on the terminal  
```bash
codename=`lsb_release -sc`
sudo sh -c "echo \"deb http://packages.osrfoundation.org/gazebo/ubuntu ${codename} main\" > /etc/apt/sources.list.d/gazebo-latest.list"
wget  http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get remove ros-${ROS_DISTRO}-simulators libsdformat1

sudo apt-get install gazebo4 ros-${ROS_DISTRO}-gazebo4-ros-pkgs ros-${ROS_DISTRO}-gazebo4-ros-control

sudo apt-get install ros-${ROS_DISTRO}-desktop ros-${ROS_DISTRO}-perception
```

The following packages can also be installed if needed  
```bash
sudo apt-get install ros-${ROS_DISTRO}-controller-manager ros-${ROS_DISTRO}-ackermann-msgs ros-${ROS_DISTRO}-effort-controllers ros-${ROS_DISTRO}-joint-state-controller ros-${ROS_DISTRO}-map-server ros-${ROS_DISTRO}-robot-pose-ekf
```

add the following line to ~/.bashrc if not already there  
```bash
source /usr/share/gazebo/setup.sh
```

