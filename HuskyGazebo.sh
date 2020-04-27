#!/bin/bash

export Local_IP=$(hostname -I)
export ROS_IP=${Local_IP// /}
export ROS_MASTER_URI=http://${Local_IP// /}:11311

export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro

SVGA_VGPU10=0 roslaunch husky_gazebo husky_empty_world.launch world_name:=worlds/smile.world laser_enabled:=false kinect_enabled:=true

