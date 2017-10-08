#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        #rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
	self.global_waypoints = None

        rospy.spin()

    def pose_cb(self, msg):
        if self.global_waypoints is None:
            return
        next_waypoints_start_num = self.find_next_waypoint(msg.pose)
	final_waypoints = Lane()
	
        rospy.loginfo("Pose Location (%s, %s), Next Waypoint Location (%s, %s)",
		msg.pose.position.x,
		msg.pose.position.y,
		self.global_waypoints.waypoints[next_waypoints_start_num].pose.pose.position.x,
		self.global_waypoints.waypoints[next_waypoints_start_num].pose.pose.position.y)

	next_waypoints_end_num = next_waypoints_start_num + LOOKAHEAD_WPS

	if next_waypoints_end_num > len(self.global_waypoints.waypoints)-1:
            next_waypoints_end_num = len(self.global_waypoints.waypoints)-1
        for i in range(next_waypoints_start_num, next_waypoints_end_num):
            final_waypoints.waypoints.append(self.global_waypoints.waypoints[i])
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
        # Below code goes in infinite loop, comment out for now
        """
        lower_wp_num = 0
        higher_wp_num = len(self.global_waypoints.waypoints) - 1

        nearest_waypoint_num = higher_wp_num
        closest_dist = None

        while 1:

            mid_wp_num = (lower_wp_num + higher_wp_num) // 2

            dist_lower = dl(self.global_waypoints.waypoints[lower_wp_num].pose.pose.position, pose.position)
            dist_upper = dl(self.global_waypoints.waypoints[higher_wp_num].pose.pose.position, pose.position)
            dist_mid = dl(self.global_waypoints.waypoints[mid_wp_num].pose.pose.position, pose.position)

            if dist_mid < closest_dist:
                closest_dist = dist_mid
                nearest_waypoint_num = mid_wp_num
            elif dist_upper < closest_dist:
                closest_dist = dist_upper
                nearest_waypoint_num = higher_wp_num
	    else:
                closest_dist = dist_lower
                nearest_waypoint_num = lower_wp_num

            # converge when all waypoinmts are next to each other
            if (lower_wp_num + 1) == mid_wp_num and (mid_wp_num + 1) == higher_wp_num:
                break
        return nearest_waypoint_num
        """

    def find_next_waypoint(self, pose):

        nearest_waypoint_num = self.find_nearest_waypoint(pose)
        #Below code does not run, comment out for now
        """
        waypoint_x = self.global_waypoints.waypoints[nearest_waypoint_num].pose.pose.position.x
        waypoint_y = self.global_waypoints.waypoints[nearest_waypoint_num].pose.pose.position.y

        heading = math.atan2(waypoint_y - pose.position.y, waypoint_x - pose.position.x)

        def quat_array(o):
            return np.array([o.x, o.y, o.z, o.w])

        (roll, pitch, yaw) = euler_from_quaternion(quat_array(pose.orientation))

        if (abs(yaw-heading) > math.pi / 4):
            nearest_waypoint_num += 1
        """
        return nearest_waypoint_num

    def waypoints_cb(self, waypoints):
        self.global_waypoints = waypoints
        

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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
