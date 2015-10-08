#!/usr/bin/env bash
if grep -Fq "source $(dirname "$PWD")/devel/setup.bash" ~/.bashrc
then
    echo Currently sourcing catkin setup.bash. OK!
else
    echo Not currently sourcing catkin setup.bash Setting..	
    echo "
source $(dirname "$PWD")/devel/setup.bash" >> ~/.bashrc
	if grep -Fq "source $(dirname "$PWD")/devel/setup.bash" ~/.bashrc
	then 
    	echo Setting successful. Open a new terminal once install is done.
	else
   	 echo Not able to set. Check your ~/.bashrc to see if really not set.
	fi
fi

if grep -Fq "export ROS_PACKAGE_PATH=$(dirname "$PWD")/src/eklavya2015-ros-pkg/eklavya_gazebo" ~/.bashrc
then
    echo  auro_resources in ROS_PACKAGE_PATH is set. OK!
else
    echo auro_resources in ROS_PACKAGE_PATH is not set. Setting...	
    echo "
export ROS_PACKAGE_PATH=$(dirname "$PWD")/src/eklavya2015-ros-pkg/eklavya_gazebo:"'$ROS_PACKAGE_PATH' >> ~/.bashrc
	if grep -Fq "export ROS_PACKAGE_PATH=$(dirname "$PWD")/src/eklavya2015-ros-pkg/eklavya_gazebo" ~/.bashrc
	then
    	echo Setting successful. Open a new terminal once install is done.
	else
   	echo Not able to set. Check your ~/.bashrc to see if really not set.
	fi
fi

if grep -Fq "export GAZEBO_RESOURCE_PATH=$(dirname "$PWD")/src/eklavya2015-ros-pkg/eklavya_gazebo" ~/.bashrc
then
    echo  auro_resources in GAZEBO_RESOURCE_PATH is set. OK!
else
    echo auro_resources in GAZEBO_RESOURCE_PATH is not set. Setting...	
    echo "
export GAZEBO_RESOURCE_PATH=$(dirname "$PWD")/src/eklavya2015-ros-pkg/eklavya_gazebo:"'$GAZEBO_RESOURCE_PATH' >> ~/.bashrc
	if grep -Fq "export GAZEBO_RESOURCE_PATH=$(dirname "$PWD")/src/eklavya2015-ros-pkg/eklavya_gazebo" ~/.bashrc
	then
    	echo Setting successful. Open a new terminal once install is done.
	else
   	echo Not able to set. Check your ~/.bashrc to see if really not set.
	fi
fi

