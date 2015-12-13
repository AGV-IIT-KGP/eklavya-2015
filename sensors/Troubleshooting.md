Troubleshooting Lidar:

Running lidar:
step 1: run roscore.
step 2: run sudo chmod 777 /dev/tty*
step 3: run rosrun hokuyo_node hokuyo_node
step 4: run step 3 in sudo and try

In case of some arbit errors,
step 1: If err no 16: wait for response.
step 2: check connections, green light should be on as long as power is being supplied.
        if green light is absent, check the junction in the lidar connection.
step 3: remove and insert the usb connection and try again.
step 4: type 'lsusb' and check if Lidar shows up in usb list. If not, load with power supply/ usb port/usb connection.
step 5: type 'rosclean purge' to remove logs and previous failures. Try again from step 1.

step 6: If nothing works, restart and try again.
