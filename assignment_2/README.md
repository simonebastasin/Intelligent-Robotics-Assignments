# README #

**[WITHOUT EXTRA POINTS IMPLEMENTATION]**

To properly execute the project we used 8 cmd sheels as follow:

> 1) roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full_tables

> 2) roslaunch tiago_iaslab_simulation apriltag.launch

> 3) roslaunch tiago_iaslab_simulation navigation.launch

> 4) rosrun tiago_iaslab_simulation human_node

> 5) rosrun tiago_iaslab_simulation proxy

> 6) rosrun tiago_iaslab_simulation nodeC

> 7) rosrun tiago_iaslab_simulation nodeB

> 8) rosrun tiago_iaslab_simulation nodeA

To help ourselfs we also used other views provided by the following commands on cmd sheels:

> 9) rosrun look_to_point look_to_point

> 10) rviz
(adding the view of the PlanningScene)

<br />

**[WITH EXTRA POINT IMPLEMENTATION]**

To execute the project considering also the extra points it is the same as before but without cmd sheel '8)' and instead execute also:

> 8) rosrun tiago_iaslab_simulation nodeA_bonus

> 11) rosrun tiago_iaslab_simulation laserScan

<br />

*[GROUP NOTES]*

All the code was written by 6 hands using Simone's and Alessandro's PCs and all members contributed equally to the work. The video in the Google Drive folder shows the robot performing the pick and place routines in the order requested by human node.