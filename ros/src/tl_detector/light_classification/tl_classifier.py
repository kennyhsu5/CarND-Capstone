from styx_msgs.msg import TrafficLight
import PIL.Image as Image
import PIL.ImageDraw as ImageDraw
import PIL.ImageFont as ImageFont
import tensorflow as tf
import rospkg
import cv2
import rospy
import numpy as np

class TLClassifier(object):
    def __init__(self):
    	rospack = rospkg.RosPack()
	self.save_file = str(rospack.get_path('tl_detector'))+'/light_classification/frozen_inference_graph.pb'
	self.detection_graph = tf.Graph()
	
	with self.detection_graph.as_default():
	    graph_def = tf.GraphDef()
	    with tf.gfile.GFile(self.save_file, 'rb') as fid:
		serialized_graph = fid.read()
		graph_def.ParseFromString(serialized_graph)
		tf.import_graph_def(graph_def, name='')

	self.sess = tf.Session(graph=self.detection_graph)
	self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
	self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
	self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
	self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
	self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
	self.debug_image = None

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
	light = TrafficLight.UNKNOWN
	image_input = np.expand_dims(image, axis=0)

	with self.detection_graph.as_default():
	    (boxes, scores, classes, num_detection) = self.sess.run(
		[self.detection_boxes, self.detection_scores,
		 self.detection_classes, self.num_detections],
		feed_dict={self.image_tensor: image_input})

	boxes = np.squeeze(boxes)
	scores = np.squeeze(scores)
	classes = np.squeeze(classes).astype(np.int32)	 

	threshold = 0.5
	lights = {
	   TrafficLight.RED: [0],
	   TrafficLight.YELLOW: [0],
	   TrafficLight.GREEN: [0]  
	}
	for i in range(num_detection):
	    score = scores[i]
	    if score > threshold:
		label = classes[i]

		if label == 1:
		    lights[TrafficLight.RED].append(score)
		elif label == 2:
		    lights[TrafficLight.YELLOW].append(score)
		elif label == 3:
		    lights[TrafficLight.GREEN].append(score)

	#self.debug_image = self.visualize(image, boxes, scores, classes)

	max_avg_score = 0
	for key in lights.keys():
	    score = np.mean(lights[key])
	    if score > max_avg_score:
		light = key
		max_avg_score = score

	return light

    def visualize(self, image, boxes, scores, classes):
	img_pil = Image.fromarray(image)
	draw_img = ImageDraw.Draw(img_pil)
	w, h = img_pil.size
	for i in range(len(classes)):
	    if scores[i] < 0.5:
		continue
	    (ymin, xmin, ymax, xmax) = boxes[i, 0], boxes[i, 1], boxes[i, 2], boxes[i, 3]
	    (left, right, top, bottom) = xmin * w, xmax * w, ymin * h, ymax * h 

	    draw_img.line([(left, top), (left, bottom), (right, bottom), (right, top), (left, top)], width=2, fill='red')

	    try:
    	        font = ImageFont.truetype('arial.ttf', 24)
  	    except IOError:
    	        font = ImageFont.load_default()

	    if classes[i] == 1:
	    	ds = 'red'
	    elif classes[i] == 2:
		ds = 'yellow'
	    elif classes[i] == 3:
		ds = 'green'
	    else:
		ds = 'unknown'
	    ds_height = font.getsize(ds)[1]
	    total_display_str_height = (1 + 2 * 0.05) * ds_height
	    if top > total_display_str_height:
    		text_bottom = top
  	    else:
    		text_bottom = bottom + total_display_str_height

	    text_width, text_height = font.getsize(ds)
    	    margin = np.ceil(0.05 * text_height)
    	    draw_img.rectangle(
        	[(left, text_bottom - text_height - 2 * margin), (left + text_width,
                                                          text_bottom)],
        	fill='red')
    	    draw_img.text(
        	(left + margin, text_bottom - text_height - margin),
        	ds,
        	fill='black',
        	font=font)


	np.copyto(image, np.array(img_pil))
	return image
