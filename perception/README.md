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

Step #1) run "roslaunch lane_detector perception.launch"


IPT Caliberation:-
Step #1) Run the lane_detecture module.launch with debug mode=5 along with the logitech_camera module.launch
Step #2) On the "Original Image window", click the 4 corners of a square on the flex in the following order, (top left, top right, bottom left,
	       bottom right)
Step #3) take the distance between the centre of the square and the mid point of the rear end of the bot. put this value in place of d= in the file
         ipt_offsets1 in data folder in lane_detector.


Debug modes:-
						Use debug mode "0" when running the actual bot.
						Use debug mode "5" for ipt caliberation
						Use debug mode "8" for normal testing

						Use time_functions = 2 for fps calculation, 0 otherwise.

while running the program in the finals, comment out all the "cv::imshow" and "cv::namedWindow" lines in the lane_navigator.cpp and logitech_camera.cpp and use debug mode 0 in lane_detector module.launch.
