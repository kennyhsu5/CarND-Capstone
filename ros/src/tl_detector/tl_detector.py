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

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

	self.store_light_wp = []	
	self.wp_closest = None     
	self.l_car_position = None
	
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
	#rospy.logerr("Traffic loaded")
	
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)       
          
        #self.store_light_wp = []
        
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

	#rospy.logerr("State: " + str(state))

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

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
	#rospy.logerr(str(pose))
	minimum = 999.0
        wp_closest = self.wp_closest
	if self.waypoints is not None:
            w_points = self.waypoints.waypoints 	    
            mininum = self.difference(pose.position.x, pose.position.y, w_points[0].pose.pose.position)
            for i, point in enumerate(w_points):
                d = self.difference_1(pose.position, w_points[i].pose.pose.position)  
                if d<minimum:
                    minimum = d
                    wp_closest = i
            self.wp_closest = wp_closest                      
        return wp_closest

    def difference(self, p1_x,p1_y, p2):
	#rospy.logerr("d")
	#rospy.logerr(str(type(p1)))
	p1_x1 = p1_x
	p1_y1 = p1_y
	x_2 = (p1_x1-p2.x)*(p1_x1-p2.x)
        y_2 = (p1_y1-p2.y)*(p1_y1-p2.y)     
        sqrt1 = math.sqrt(x_2+y_2)         
        return sqrt1

    def difference_1(self, p1, p2):
	x_2 = (p1.x-p2.x)*(p1.x-p2.x)
        y_2 = (p1.y-p2.y)*(p1.y-p2.y)     
        sqrt1 = math.sqrt(x_2+y_2) 
        #rospy.logerr("p1: " +str(p1) + " p2: "+str(p2))
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

        x, y = self.project_to_image_plane(light.pose.pose.position.x,light.pose.pose.position.y)
	#x, y = self.project_to_image_plane(light[0], light[1])

        #TODO use light location to zoom in on traffic light in image
	img = cv2.resize(cv_image, (300,200))

        #Get classification
        return self.light_classifier.get_classification(img)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

	 # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            if car_position is not None:
                self.l_car_position = car_position

	#l = ""
	#light_position = self.config
	#for key in self.config.keys():
	#	l = l+str(key)		
	#rospy.logerr(str(l))

	light_position = self.lights
	#rospy.logerr(str(light_position[0].pose.pose.position.x))

	#Assign lights to waypoints
        light_wp = []	
	if self.waypoints is not None:
	    waypoints = self.waypoints	
	    for i in range(len(light_position)):
	        position = self.light_get_closest_waypoint(waypoints, light_position[i])    
		light_wp.append(position)
            self.store_light_wp = light_wp 
	else:            
            light_wp = self.store_light_wp       
		 
        # Find the closest visible traffic light (if one exists)
	# Find position of closest light
	if len (light_wp):	
	    if (self.l_car_position > max (light_wp)):
                light_wp_num = min(light_wp)
            else: 
                a = light_wp[:]
   	        a[:] = [x-self.l_car_position for x in a]
                light_wp_num = min(i for i in a if i>0)
                light_wp_num = light_wp_num + self.l_car_position            

	        index1 = light_wp.index(light_wp_num) 	
                light = light_position[index1]
        
	# Check only if light is in certain distance 150m:           
	#rospy.logerr(str(type(light)))

        if light:
            distance = self.difference(light.pose.pose.position.x, light.pose.pose.position.y, self.waypoints.waypoints[self.l_car_position].pose.pose.position) 
	    if distance >150:
                return -1, TrafficLight.UNKNOWN	
            else:
        	state = self.get_light_state(light)
		#rospy.logerr(str(state))
                return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

    def light_get_closest_waypoint(self, wp, light_pos):

        """Identifies the closest path waypoint to a given light
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem

        """	        
        wp_closest = None 
        points = wp.waypoints 
        mininum = self.difference_l(light_pos.pose.pose.position, points[0].pose.pose.position)
        for i, point in enumerate(points):
            d = self.difference_l(light_pos.pose.pose.position, point.pose.pose.position)  
            if d<mininum:
                mininum = d
                wp_closest = i            
        return wp_closest
    
    def difference_l(self, light_po, p2):
	#rospy.log(str(light_po))
	x_2 = (light_po.x-p2.x)*(light_po.x-p2.x)
        y_2 = (light_po.y-p2.y)*(light_po.y-p2.y)     
        sqrt1 = math.sqrt(x_2+y_2) 
        return sqrt1
	

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
