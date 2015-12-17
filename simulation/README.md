# agv_simulation

Simulator for new bot.

Sensors added and the topics they publish to:
*	Camera : /eklavya/image_raw
*	Lidar  : /eklavya/laser/scan
*	IMU   :  /eklavya/imu
*	GPS   : /eklavya/gps/fix


Files where respective sensor topics are defined:
*	Camera : eklavya5_description/robot/chasisss.urdf.xacro
*	Lidar: eklavya5_description/robot/chasisss.urdf.xacro
*	IMU : eklavya5_description/sensors/imusensor_gazebo.urdf.xacro
*	GPS : eklavya5_description/sensors/hector_gps_gazebo.urdf.xacro


Note: Plugin for GPS is a custom plugin, hence the package "eklavya5_gazebo_plugins" has to be built for the GPS to work.


The simulator is launch by:

roslaunch eklavya5_gazebo eklavya_igvc_final.launch

