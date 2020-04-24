
Local_IP=$(hostname - I)
Remote_IP=111.111.111.111 #Master IP
export ROS_IP=${Local_IP// /} 
export ROS_MASTER_URI=http://${Remote_IP}:11311
