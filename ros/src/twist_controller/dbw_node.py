#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped, Pose, Point
from styx_msgs.msg import Lane
import numpy as np
import math
import tf


# For path visulization
from nav_msgs.msg import Odometry

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        max_throttle    = 0.5


        self.vis_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.odom = Odometry()


        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        self.controller = Controller(vehicle_mass, fuel_capacity, brake_deadband,
                decel_limit, accel_limit, wheel_radius,
                wheel_base, steer_ratio, max_lat_accel,
                max_steer_angle, max_throttle) 
        # TODO: Subscribe to all the topics you need to
        self.twist_cmd        = None
        self.is_dbw_enabled   = None
        self.current_velocity = None
        self.currTime         = rospy.get_time()
        self.lastaction       = None
        self.final_waypoints  = None
        self.current_pose     = None

        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb, queue_size=1)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)
        rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb, queue_size=1)
        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            currTime = rospy.rostime.get_time()
            elapsed = currTime - self.currTime
            self.currTime = currTime
            
            if self.current_velocity is not None and self.twist_cmd is not None:
                #cte = self.calculateCTE(self.current_pose, self.final_waypoints)
                cte = None
                throttle, brake, steering = self.controller.control(self.twist_cmd.twist.linear.x,
                                                self.twist_cmd.twist.angular.z,
                                                self.current_velocity.twist.linear.x,
                                                self.current_velocity.twist.angular.z,
                                                self.is_dbw_enabled,
                                                elapsed,
                                                cte)
                self.odom.header.stamp = rospy.Time.now()
                self.odom.header.frame_id= "odom"
                self.odom.pose.pose = self.current_pose
                self.odom.child_frame_id = "base_link"
                self.vis_pub.publish(self.odom)

                if self.is_dbw_enabled:
                    #publish the control command only if dbw is enabled
                    action = 'brake' if brake > 0.0 else 'throttle'
                    if action == self.lastaction:
                        if action == 'brake':
                            throttle = None
                        else:
                            brake = None
                    self.lastaction = action
                    self.publish(throttle, brake, steering)        
            rate.sleep()

    def calculateCTE(self, current_pose, final_waypoints):
        if current_pose is None or final_waypoints is None:
            return None

        _, _, angle = tf.transformations.euler_from_quaternion([current_pose.orientation.x,
                    current_pose.orientation.y,
                    current_pose.orientation.z,
                    current_pose.orientation.w])

        xorigin = current_pose.position.x
        yorigin = current_pose.position.y 
        waypoints_matrix = np.array([ [pt.pose.pose.position.x - xorigin, 
            pt.pose.pose.position.y - yorigin] for pt in final_waypoints])
        rotationMatrix = np.array([
                [np.cos(angle), -np.sin(angle)],
                [np.sin(angle), np.cos(angle)]
               ])

        rotated = np.dot( waypoints_matrix, rotationMatrix)
        coefficients = np.polyfit(rotated[:, 0], rotated[:, 1], 2)
        #target = np.polyval(np.polyder(coefficients, 1), 0)
        #target = math.atan(target)
        #return target
        x = rotated[1, 0]
        target = np.polyval(np.polyder(coefficients, 1), x)
        return math.atan(target)
        #return math.atan(np.polyval(coefficients, 10))

    def publish(self, throttle, brake, steer):
        if throttle is not None:
            tcmd = ThrottleCmd()
            tcmd.enable = True
            tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
            tcmd.pedal_cmd = throttle
            self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        if brake is not None:
            bcmd = BrakeCmd()
            bcmd.enable = True
            bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
            bcmd.pedal_cmd = brake
            self.brake_pub.publish(bcmd)

    def twist_cb(self, msg):
        self.twist_cmd = msg

    def dbw_cb(self, msg):
        if msg is not None:
            self.is_dbw_enabled = msg.data

    def velocity_cb(self, msg):
        self.current_velocity = msg

    def final_waypoints_cb(self, msg):
        self.final_waypoints = msg.waypoints

    def current_pose_cb(self, msg):
        self.current_pose = msg.pose

if __name__ == '__main__':
    DBWNode()
