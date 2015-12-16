This package (apriltags_ros) was originally develloped in:
1. https://travis-ci.org/RIVeR-Lab/apriltags_ros
2. https://github.com/Humhu/apriltags.git
and was further modified from Dimitrios Kanoulas and tested on Ubuntu 14.04LTS.
=============

This is the AprilTags implementation for ROS, using either:
1. the CMU Multisense-SL/Multisense-S7 sensor
2. the ASUS Xtion LIVE Pro sensor.

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

Run (AprilTag detector, with ASUS, using the OpenNi driver):
-- roscore
-- roslaunch openni2_launch openni2.launch depth_registration:=true
-- rviz
-- roslaunch apriltags_ros detector_openni.launch

=============
The ASUS was not able to work in 30Hz for the AprilTags detection due to
various reasons. A survey on various grabbers using openni/openni2
drivers is as follows:
1. Openni
When using openni:
> roslaunch openni_launch openni.launch depth_registration:=true
it has been noted that subscribing in Image and Point Cloud data does not
work in real-time (30Hz).  Thus even having 2 callback functions, one for
the RGB and one for the Depth/Point image/cloud not synced, synced, or
approximately synced is not working in 30Hz.  Also using the PCLOpenniGrabbers
does not help with that because it is assumed that the above works (pcl just
makes a good implementation of an approximate synced callbacks).

2. Openni2
When using openni2:
> roslaunch openni2_launch openni2.launch depth_registration:=true
it has been noted that PCLOpenni2Grabbers are not working at all, because
ASUS cannot open with USB3.0 ports.  When (approx) synced image/depth callbacks
were used then again it was not working in 30Hz.  Thus we have tried to make it
work by extracting the RGB image from an organized point cloud.  The it has
been noted that the cc code of apriltag detector needs more time to work on
this image than the image callback one (!).  For now the code works very slowly
100ms to detect Apriltags and needs improvement.
