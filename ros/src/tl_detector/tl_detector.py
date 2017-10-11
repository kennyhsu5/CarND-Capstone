#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np
from scipy.misc import imsave
import time

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
	
	self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
	
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)       
        
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
	if self.waypoints is None:
	    return -1

	minimum = 99999.0
	wp_closest = -1
        for i, point in enumerate(self.waypoints.waypoints):
            d = self.difference(x, y, point.pose.pose.position.x, point.pose.pose.position.y)  
            if d < minimum:
                minimum = d
                wp_closest = i                      
        return wp_closest

    def difference(self, p1x, p1y, p2x, p2y):
	x_diff = (p1x-p2x)*(p1x-p2x)
        y_diff = (p1y-p2y)*(p1y-p2y)     
        sqrt1 = math.sqrt(x_diff+y_diff) 
        return sqrt1
	

    def project_to_image_plane(self, point_in_world1, point_in_world2):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #Use tranform and rotation to calculate 2D position of light in image	
        #Used as reference point for camera calibration
	c_x = point_in_world1
        c_y = point_in_world2

        #c_x = point_in_world[0]
        #c_y = point_in_world[1]

        #rospy.logerr ("point x: " + str(c_x) + "point y: "+ str(c_y))
        
        #Transformation of coordinate system to a car coordinate system
        #http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
        #http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
		
        x = self.pose.pose.orientation.x
        y = self.pose.pose.orientation.y
        z = self.pose.pose.orientation.z
        w = self.pose.pose.orientation.w
	
	#rospy.logerr ("Pose x: " + str(x))	

        #heading = atan2(2*qy*qw-2*qx*qz , 1 - 2*qy2 - 2*qz2)
        val1 = 2.0*y*w-2.0*x*z
        val2 = 1.0 - 2.0*y*y - 2.0*z*z	
        heading = math.degrees(math.atan2(val1, val2))

        #rospy.logerr ("Heading: " + str(heading))	
        
        #define coordinates (origin)

	x_1 = (c_y-self.pose.pose.position.y)*math.sin(math.radians(heading))-(self.pose.pose.position.x- c_x)*math.cos(math.radians(heading))
	y_1 = (c_y-self.pose.pose.position.y)*math.cos(math.radians(heading))-(c_x-self.pose.pose.position.x)*math.sin(math.radians(heading))


	#x_1 = 0
        #y_1 = 0
        
        objectPoints = np.array([[float(x_1), float(y_1), 0]],dtype = np.float32)
        rvec = (0,0,0) 	
        tvec = (0,0,0) 	
        cameraMatrix = np.array([[fx,0,image_width/2],
                                [0,fy,image_height/2],  	
                                 [0,0,1]])
        distCoeffs = None
        ret,_ = cv2.projectPoints(objectPoints, rvec,tvec,cameraMatrix, distCoeffs)	

        x2 = int(ret[0,0,0])
        y2 = int(ret[0,0,1])

	#rospy.logerr("x: "+str(x2))
        return (x2, y2)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
	img = cv2.resize(cv_image, (300,200))
        tmp = self.light_classifier.get_classification(img)
	#imsave(str(tmp) + "-" + str(time.time()) + ".png", img)
	return tmp

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
	if self.waypoints is None:
	    return -1, TrafficLight.UNKNOWN
        light = None

	 # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

	min_pos_diff = 99999
	state, tmp = 4, 0
	for pos in stop_line_positions:
	    # Better to compute distanc to stop line based on coordinates, but we are given position so
	    # use that for now.
	    light_wp_position = self.get_closest_waypoint(pos[0], pos[1])
	    diff = light_wp_position - car_position
	    # Only use stop lines that are ahead in waypoint position of the car.
	    # Edge case when the track wraps around need to be handled later
	    if diff > 0 and diff < min_pos_diff:
		min_pos_diff = diff
		light = light_wp_position
		state = self.lights[tmp].state
	    tmp += 1

        if light:
            #state = self.get_light_state(light)
            return light, state
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
