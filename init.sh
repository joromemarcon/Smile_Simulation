#!/bin/bash

#####################################################
#Edit Gazebo_Install_Path to your install directory #
#####################################################

Gazebo_Install_Path=/usr/share/gazebo-9
Worlds=${Gazebo_Install_Path}/worlds
Mat_Scripts=${Gazebo_Install_Path}/media/materials/scripts/
Mat_Textures=${Gazebo_Install_Path}/media/materials/textures/

cp worlds/* $Worlds
cp media/materials/scripts/* $Mat_Scripts
cp media/materials/textures/* $Mat_Textures

source ${Gazebo_Install_Path}/setup.sh
