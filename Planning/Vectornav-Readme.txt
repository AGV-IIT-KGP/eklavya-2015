Launch /vn_ins/launch/module.launch. (Takes time to catch satellite data)

The node output can be seen on the following topics:
1. /vn_ins/fix (GPS)
2. /vn_ins/Imu (IMU data)
3. /vn_ins/Twist (linear and angular velocities)

The green light on the USB cable indicates connection to the computer.
The red light indicates data transfer between the computer and the vectornav. It should start blinking after launching "module.launch". If it doesn't check that it is connected by using "ls /dev/serial/by-id" on the terminal. The ID is "/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_FTVJUC0O-if00-port0". If it is connected and still the red light is not blinking, use "sudo chmod 777 /dev/serial/by-id/usb-FTDI_USB-RS232_Cable_FTVJUC0O-if00-port0".

Further issues can be debugged using Cutecom. (to install - "sudo apt-get install cutecom") 
Always launch cutecom in sudo mode.
The device is set to output Yaw,Pitch,Roll under the $VNYPR header at 40Hz.
This can be set by sending the following commands through cutecom:
1. $VNWRG,06,1*6D (sets Asynchronous output mode to YPR, the output header should change to $VNYPR)
2. $VNWRG,67,0,0,0,0*77 (sets the AHRS mode required for YPR data)
3. $VNWRG,07,40*59 (sets the output data rate to 40Hz)
4. $VNRRG,58*7e (reads GPS data)
5. $VNWNV*57 (writes/saves the above settings to the device)
