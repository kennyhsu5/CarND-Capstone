#!/usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
STOP_DISTANCE = 5
DECEL_SPEED = 0.20

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
	rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

	self.global_waypoints = None
	self.red_light_wp = -1

        rospy.spin()

    def pose_cb(self, msg):
        if self.global_waypoints is None:
            return
        next_waypoints_start_num = self.find_next_waypoint(msg.pose)
	final_waypoints = Lane()

	next_waypoints_end_num = next_waypoints_start_num + LOOKAHEAD_WPS
	slow_down = False
	if next_waypoints_end_num > len(self.global_waypoints.waypoints)-1:
            next_waypoints_end_num = len(self.global_waypoints.waypoints)-1
	elif self.red_light_wp != -1 and next_waypoints_end_num > self.red_light_wp:
	    next_waypoints_end_num = self.red_light_wp - STOP_DISTANCE
	    slow_down = True

        for i in range(next_waypoints_start_num, next_waypoints_end_num):
            final_waypoints.waypoints.append(self.global_waypoints.waypoints[i])
	if slow_down:
	    if len(final_waypoints.waypoints) > 1:
	        curr_vel = self.get_waypoint_velocity(final_waypoints.waypoints[0])
	        num_wp_to_stop = len(final_waypoints.waypoints)
	        steps_to_stop = min(int(curr_vel / DECEL_SPEED), num_wp_to_stop)
	        speed_target = 0
	        for i in range(1, steps_to_stop):
		    self.set_waypoint_velocity(final_waypoints.waypoints, num_wp_to_stop - i, speed_target)
		    speed_target += DECEL_SPEED
	else:
	    for i in range(len(final_waypoints.waypoints)):
	    	# Set to original velocity
	    	self.set_waypoint_velocity(final_waypoints.waypoints, i, self.get_waypoint_velocity(self.global_waypoints.waypoints[0]))
	self.final_waypoints_pub.publish(final_waypoints)

    def find_nearest_waypoint(self, pose):
        def dl(a, b):
            return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)

        tmp_dist = 1000000
        tmp_i = None
        for i in range(len(self.global_waypoints.waypoints)):
            position = self.global_waypoints.waypoints[i].pose.pose.position
            dist = dl(position, pose.position)
            if dist < tmp_dist:
               tmp_dist = dist
               tmp_i = i

        return tmp_i

    def find_next_waypoint(self, pose):

        nearest_waypoint_num = self.find_nearest_waypoint(pose)
        waypoint_x = self.global_waypoints.waypoints[nearest_waypoint_num].pose.pose.position.x
        waypoint_y = self.global_waypoints.waypoints[nearest_waypoint_num].pose.pose.position.y

        heading = math.atan2(waypoint_y - pose.position.y, waypoint_x - pose.position.x)

        def quat_array(o):
            return np.array([o.x, o.y, o.z, o.w])

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quat_array(pose.orientation))

        if (abs(yaw-heading) > math.pi / 4):
            nearest_waypoint_num += 1
        return nearest_waypoint_num

    def waypoints_cb(self, waypoints):
        self.global_waypoints = waypoints
        

    def traffic_cb(self, msg):
	self.red_light_wp = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
