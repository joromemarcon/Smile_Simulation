#!/bin/bash

rostopic pub /husky_velocity_controller/cmd_vel geometry_msgs/Twist "linear:
  x: 5.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"

