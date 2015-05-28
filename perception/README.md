Steps to train and use lane_detector.

Training:-

Step #1) An image/images of the lanes to be trained on must be present in the "data" 
		 directory (~/catkin_ws/src/eklavya-2015/perception/lane_detector/data/)
Step #2) Edit step1.sh (follow instructions inside)
Step #3) Run step1.sh (Run it with a dot, i.e. execute ". step1.sh" ,without quotes)
Step #4) Edit step2.sh (follow instructions inside)
Step #4) Run step2.sh in the same format as step1 (You need to go back to that directory first)
Step #5) The launch file opens up in gedit. Go to parameter "training_data_file" and set value equal 
		to the trained file name.

Using lane_detector:-

Step #1) run "roslaunch logitech_cammera module.launch"
Step #2) run "roslaunch lane_detector module.launch"
Step #3) run "roslaunch lane_navigator module.launch"
