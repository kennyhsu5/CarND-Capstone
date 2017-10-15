from styx_msgs.msg import TrafficLight
from tensorflow.contrib.layers import flatten
import tensorflow as tf
import rospkg
import cv2
import rospy
import numpy as np

class TLClassifier(object):
    def __init__(self):
    	rospack = rospkg.RosPack()
	self.save_file = str(rospack.get_path('tl_detector'))+'/light_classification/model.ckpt'
	tf.reset_default_graph()
	self.network = self.ConvNet()
	self.sess = tf.Session()
	self.saver = tf.train.Saver()
	self.saver.restore(self.sess, self.save_file)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
	label_to_signal = {
	    0: TrafficLight.RED,
	    1: TrafficLight.YELLOW,
	    2: TrafficLight.GREEN,
            3: TrafficLight.UNKNOWN,
	}
	image = image.reshape(1, 64, 96, 3)
	label = self.sess.run(self.network.inference, feed_dict={
	    self.network.x: image,
	    self.network.keep_prob: 1.0
	})[0]
	return label_to_signal[label]

    class ConvNet:
        def __init__(self):
            self.x = tf.placeholder(tf.float32, shape=[None, 64, 96, 3], name='x')
            self.y = tf.placeholder(tf.float32, shape=[None, 4], name='y')
            self.keep_prob = tf.placeholder(tf.float32, name='keep_prob')
        
            # Layers of NN
	    self.layer1 = self.conv_layer(self.x, 3, 4, 'layer1')
            self.layer1_pool = tf.nn.max_pool(self.layer1, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')
            self.layer2 = self.conv_layer(self.layer1_pool, 3, 4, 'layer2')
            self.layer2_pool = tf.nn.max_pool(self.layer2, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')
    
            self.layer3 = self.conv_layer(self.layer2_pool, 3, 8, 'layer3')
            self.layer3_pool = tf.nn.max_pool(self.layer3, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')
            self.layer4 = self.conv_layer(self.layer3_pool, 3, 8, 'layer4')
            self.layer4_pool = tf.nn.max_pool(self.layer4, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')
        
            flatten_size = 4 * 6 * 8
            self.flatten = tf.reshape(self.layer4_pool, [-1, flatten_size])
            self.layer7 = self.fc_layer(self.flatten, 50, 'layer7')
            self.layer7_dropout = tf.nn.dropout(self.layer7, self.keep_prob)
            self.logits = tf.contrib.layers.fully_connected(self.layer7_dropout, 4, activation_fn=None, scope='logits') 
            self.inference = tf.argmax(self.logits, axis=1)
        
        def fc_layer(self, x, hh_size, scope):
            with tf.variable_scope(scope):
                h1 = tf.contrib.layers.fully_connected(x, hh_size, activation_fn=None, scope='fully_connected')
                return tf.nn.relu(h1, 'relu')
        
        def conv_layer(self, x, kernel_size, filter_size, scope):
            with tf.variable_scope(scope):
                h1 = tf.contrib.layers.conv2d(x, filter_size, kernel_size, activation_fn=None, scope='convolution')
                return tf.nn.relu(h1, 'relu')

