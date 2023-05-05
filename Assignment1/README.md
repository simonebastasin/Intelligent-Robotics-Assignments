# README #

[WITH EXTRA POINTS IMPLEMENTATION]

To properly execute the project we used 5 cmd sheels as follow:
> 1) roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full
> 2) roslaunch tiago_iaslab_simulation navigation.launch
> 3) rosrun tiago_iaslab_simulation client 11 0 -45
> 4) rosrun tiago_iaslab_simulation proxyMCL
> 5) rosrun tiago_iaslab_simulation laserScan
> 6) rosrun tiago_iaslab_simulation motionLawServer

Notes:
- client.cpp accepts Pose_B coordinates as command line arguments: x y yaw_degrees
(this arguments are given in starting robot reference frame and yaw_degrees is given in degrees)

Conslusions:
Using these commands, from the extimated Pose_B of the example image in the assignment {x=11, y=0, yaw=-45°} we found the following obastacle coordinates (w.r.t. world reference frame):
- {(3.82, -0.31), (4.54, -2.49), (5.99, -1.44), (5.86, 0.84)}
That compared with ground truth coordinates given by Gazebo:
- {(4.16, -0.30), (4.87, -2.42), (6.16, -1.35), (5.81, 0.76)}
We found the mean square error: M.S.E. = 0.2754


[WITHOUT EXTRA POINTS IMPLEMENTATION]

To execute the project without considering the extra points it's the same as before but without cmd sheel '6)' and instead of
> 4) rosrun tiago_iaslab_simulation proxyMCL
to use move_base froms the beginning of the movement we used
> 4) rosrun tiago_iaslab_simulation proxy