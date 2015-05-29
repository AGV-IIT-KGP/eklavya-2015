# Controls-Eklavya4.0-PC
First copy controls, xbox and eklavya4_roboteq folders to a new folder and catkin_make

#Installing Arduino Libraries
For Using Arduino Due in Linux, you need to get the latest version of the IDE from www.arduino.cc, extract the tarball archive to a folder, cd to the folder and run ./arduino to run the IDE. For adding a new zip library, go to Tools>Import Library>Add Libraries and select the two zip files. The rosserial lib has to be installed manually which is a 5 minute tutorial at http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

#Running Files
Running order is 1. xboxdrv.sh 2. arduino IDE 1.6.4 (open serial port of arduino labelled DAC) 3.encoder_pkg.sh 4. robo_launch.sh 5. launch.sh  (Preferably all in root, works nonetheless)

#Launch Files
Main launch file for controls package is Fullcontrol.launch, has parameters for PID tuning and setting loop rates along with constraints on max, min values

#Notes
1. Pressing Red and Yellow Buttons in succession on the Xbox resets the target velocities and target curvature, and also their accumulated integral error, bringing bot to free run condition with no velocity.
2. Left top button enables manual mode, Right Top enables autonomous mode
3. To plot and view graphs of pid tuning, run 'rqt_plot /va /vt' and 'rqt_plot /wa /wt'
