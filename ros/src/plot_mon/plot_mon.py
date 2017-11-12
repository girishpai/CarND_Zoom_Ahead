#!/usr/bin/env python

import sys
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path

from styx_msgs.msg import Lane, Waypoint
from plot_tool.srv import *

import math
import csv


'''
This node will demo how to use plot_tool to draw trajectory of the simulated car.
'''


class plot_mon():
    def __init__(self, filename):
        rospy.init_node('plot_mon_node')
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.wait_for_service('plot_tool/draw_pose')

        self.skip_cnt = 0
    
        self.pose = Pose()

        # Draw /base_waypoints from <tot>/data/wp_yaw_const.csv
        print filename
        myplot = rospy.ServiceProxy('plot_tool/draw_pose', PlotPose)
        with open(filename,'rb') as csvfile:
            rd = csv.reader(csvfile, delimiter=',')
            for row in rd:
                self.skip_cnt += 1
                if self.skip_cnt > 50:
                    x = float(row[0])
                    y = float(row[1])

				    # Create empty msg
                    message = PlotPoseRequest()
                    message.msg.position.x = x
                    message.msg.position.y = y
                    message.series=np.uint32(5) # yellow
                    message.append=True
                    message.symbol=ord('t') # triangle
                    message.symbol_size=np.uint32(3)

                    myplot(message)

                    self.skip_cnt = 0
 

        self.proc_loop()
        
    
    def proc_loop(self):
        while not rospy.is_shutdown():
            # Work every 2 seconds.
            rospy.sleep(2.)
            
           
    def pose_cb(self, msg):
        self.skip_cnt += 1
        if self.skip_cnt >50:
            self.pose = msg.pose
            try:
                myplot = rospy.ServiceProxy('plot_tool/draw_pose', PlotPose)
                
				# Create empty msg
                message = PlotPoseRequest()

                # Draw latest pose
                #print self.pose.position 
                message.msg = self.pose
                message.series=np.uint32(0) # green
                message.append=True
                message.symbol=ord('o')
                message.symbol_size=np.uint32(3)
    
                myplot(message)

                # Draw car head pose in different color
                message.series=np.uint32(4) # magenta
                message.append=False
                message.symbol=ord('d')
                message.symbol_size=np.uint32(10)
     
                myplot(message)

            except rospy.ServiceException, e:
                print 'Serive call failed: %s'%e
            
            self.skip_cnt = 0
 
if __name__ == '__main__':
    try:
        plot_mon(sys.argv[1])
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start plot_mon node.')
