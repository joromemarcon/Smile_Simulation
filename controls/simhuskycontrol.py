#!/usr/bin/env python

import rospy, tf
import geometry_msgs.msg, nav_msgs.msg
from math import *
from time import sleep

class Simulator:
    def __init__(self):
        self.next_goal=0
        self.goal=[0,0]

    def listen(self, cmdmsg, cmdpub):
        rospy.Subscriber('odometry/filtered',nav_msgs.msg.Odometry,self.huskyOdomCallback, 
                 (cmdpub,cmdmsg))
        rospy.spin()

    def goal_switch(self, select):
        switch={0: [9,0], 
                1: [10,-2],
                2: [10,-5],
                3: [10, -6],
                4: [9, -7],
                5: [0, -8]}
        self.goal = switch.get(select)

    def huskyOdomCallback(self, message,cargs):
        # Implementation of proportional position control 
        # For comparison to Simulink implementation

        # Callback arguments 
        pub,msg = cargs


        # Tunable parameters
        wgain = 15.0 # Gain for the angular velocity [rad/s / rad]
        vconst = 5.0 # Linear velocity when far away [m/s]
        distThresh = 0.5 # Distance treshold [m]

        # Generate a simplified pose
        pos = message.pose.pose
        quat = pos.orientation
        # From quaternion to Euler
        angles = tf.transformations.euler_from_quaternion((quat.x,quat.y,
                                                           quat.z,quat.w))
        theta = angles[2]
        pose = [pos.position.x, pos.position.y, theta]  # X, Y, Theta 
        self.goal_switch(self.next_goal)
        # Proportional Controller
        v = 0 # default linear velocity
        w = 0 # default angluar velocity
        distance = sqrt((pose[0]-self.goal[0])**2+(pose[1]-self.goal[1])**2)
        if (distance > distThresh):
            v = vconst
            desireYaw = atan2(self.goal[1]-pose[1],self.goal[0]-pose[0])
            u = desireYaw-theta
            bound = atan2(sin(u),cos(u))
            w = min(0.5 , max(-0.5, wgain*bound))
        if (distance < distThresh):
            self.next_goal = self.next_goal + 1
            print ("NEXT, {}", self.next_goal);
        # Publish
        msg.linear.x = v
        msg.angular.z = w
        pub.publish(msg)
        
        # Reporting
        print('huskyOdomCallback: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f'%(pose[0],pose[1],distance,v,w))

########################################
# Main Script
# Initialize our node
rospy.init_node('nre_simhuskycontrol',anonymous=True)
    
# Set waypoint for Husky to drive to
next_goal=1
goal = 0 #goal_switch(next_goal)


# Setup publisher
cmdmsg = geometry_msgs.msg.Twist()
cmdpub = rospy.Publisher('/cmd_vel',geometry_msgs.msg.Twist, queue_size=10)

# Setup subscription - which implemets our controller.
# We pass the publisher, the message to publish and the goal as 
# additional parameters to the callback function.
#rospy.Subscriber('odometry/filtered',nav_msgs.msg.Odometry,huskyOdomCallback, 
#                 (cmdpub,cmdmsg,goal,next_goal))


sim = Simulator()
sim.listen(cmdmsg, cmdpub)
# spin() simply keeps python from exiting until this node is stopped
#rospy.spin()
