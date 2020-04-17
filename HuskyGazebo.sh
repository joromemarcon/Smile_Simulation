#!/bin/bash

export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro

SVGA_VGPU10=0 roslaunch husky_gazebo husky_empty_world.launch world_name:=worlds/smile.world

