from styx_msgs.msg import TrafficLight
from tensorflow.contrib.layers import flatten
import tensorflow as tf
import rospkg
import cv2
import rospy
import numpy as np

class TLClassifier(object):
    def __init__(self):

	self.det_graph = tf.Graph()

        with self.det_graph.as_default():
            self.x = tf.placeholder(tf.float32, (None, 200, 300, 3))
            self.y = tf.placeholder(tf.int32, (None))
            self.logits = self.LeNet(tf.cast(self.x, tf.float32))
            self.initialize = tf.global_variables_initializer()
            self.save = tf.train.Saver()
            self.sess1= tf.Session()
            rospack = rospkg.RosPack()
	    self.save_file = str(rospack.get_path('tl_detector'))+'/light_classification/det_model.ckpt'
            #self.save_file = str(os.path.join(os.getcwd(), 'trained_variables.ckpt'))
            self.save.restore(self.sess1, self.save_file)
	    #rospy.logerr('TF loaded')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
	color = TrafficLight.UNKNOWN

	img = self.adjust_gamma(image)
	img = img.reshape(1,200,300,3)	

        with self.det_graph.as_default():
            prediction =  self.sess1.run(self.logits, feed_dict={self.x: img})

        value = np.argmax(prediction)	
	if value == 1:
            color = TrafficLight.GREEN		
	elif value == 2:
            color = TrafficLight.YELLOW		
	elif value == 3:
            color = TrafficLight.RED	

        return color

    def adjust_gamma(self, image, gamma=2.0):
        # build a lookup table mapping the pixel values [0, 255] to
        # their adjusted gamma values
 
        invGamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** invGamma) * 255
        for i in np.arange(0, 256)]).astype("uint8")
        return cv2.LUT(image, table)


    def conv2D(self,in_layer, shape1, zeros1, mu = 0,sigma = 0.1):
        # Layer: Convolutional
        _conv1_W = tf.Variable(tf.truncated_normal(shape=shape1, mean = mu, stddev = sigma))
        _conv1_b = tf.Variable(tf.zeros(zeros1))
        _conv1   = tf.nn.conv2d(in_layer, _conv1_W, strides=[1, 1, 1, 1],padding='VALID') + _conv1_b

        # Activation.
        _conv1 = tf.nn.relu(_conv1)

        # Pooling
        _conv1 = tf.nn.max_pool(_conv1, ksize=[1, 2, 2, 1], strides=[1, 2,2, 1], padding='VALID')
        return _conv1


    def deconv2D(self,in_layer, shape1, zeros1, mu = 0,sigma = 0.1, activate=True):

        # Fully connected
        _fc1_W = tf.Variable(tf.truncated_normal(shape=shape1, mean = mu,stddev = sigma))
        _fc1_b = tf.Variable(tf.zeros(zeros1))
        _fc1   = tf.matmul(in_layer, _fc1_W) + _fc1_b
        if activate == True:
            _fc1    = tf.nn.relu(_fc1)

        return _fc1



    def LeNet(self,img):
 
        layer1 = self.conv2D(img,(5, 5, 3, 16), 16)
        layer2 = self.conv2D(layer1,(5, 5, 16, 32), 32)
        layer3 = self.conv2D(layer2,(3, 3, 32, 64), 64)

        fc0   = flatten(layer3)

        outlayer1 =  self.deconv2D(fc0,(49280, 150), 150)
        outlayer2 =  self.deconv2D(outlayer1,(150, 100), 100)
        outlayer3 =  self.deconv2D(outlayer2,(100, 50), 50)
        outlayer4 =  self.deconv2D(outlayer3,(50, 4), 4, activate=False)

        return outlayer4
	
