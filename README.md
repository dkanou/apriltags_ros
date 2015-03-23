apriltags_ros was originally develloped in https://travis-ci.org/RIVeR-Lab/apriltags_ros
=============

This is the AprilTags implementation for ROS, using the CMU Multisense sensor, as well as an ASUS Xtion LIVE Pro.

Installation:
-- mkdir apriltags_ros
-- mkdir apriltags_ros/src
-- cd apriltags_ros/src
-- git init
-- git clone https://github.com/dkanou/apriltags_ros.git
-- catkin_init_workspace
-- cd ..
-- catkin_make
-- cd ..
-- source devel/setup.bash

Run:
-- roscore
-- roslaunch openni2_launch openni2.launch depth_registration:=true
-- rviz
-- roslaunch apriltags_ros detector.launch
