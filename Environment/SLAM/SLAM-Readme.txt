Launch "SLAM/launch/final2.launch" after launching vn_ins(vectornav) and encoder nodes.
NOTE: change the "Magnetic Declination" parameter in the "Nav_Sat_Transform" node depending on the location. (instructions given in the launch file)

The node requires inputs from the following topics:
1. /vn_ins/fix (GPS)
2. /encoders
3. /vn_ins/Imu

The node outputs the following:
1. /odometry/filtered
2. /gps/filtered

If the nodes die, increase the "Sensor timeout" in the launch file.
NOTE: final.launch should not be used.
      /odometry/filtered1 topic should not be used.