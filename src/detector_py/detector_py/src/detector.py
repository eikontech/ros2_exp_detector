from rclpy.node import Node

from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import TrackedObjectsStamped
from cv_bridge import CvBridge

import cv2
import numpy as np

#global vars
contimg = 0
modelFile = 'ssd_mobilenet_frozen_inference_graph.pb'
configFile = 'ssd_mobilenet_v2_coco_2018_03_29.pbtxt'
classFile = 'coco_class_labels.txt'

import requests
from os import path

# Download model if not present in the folder.
if(not path.exists(modelFile)):
    print('Downloading MobileNet SSD Model.......')
    
    url = 'https://opencv-courses.s3.us-west-2.amazonaws.com/ssd_mobilenet_frozen_inference_graph.pb'

    r = requests.get(url)

    with open(modelFile, 'wb') as f:
        f.write(r.content)

    print('ssd_mobilenet_frozen_inference_graph Download complete!')

# Read Tensorflow network.
net = cv2.dnn.readNetFromTensorflow(modelFile, configFile)

# Check Class Labels.
with open(classFile) as fp:
    labels = fp.read().split('\n')
print(sorted(labels))

# Detect Objects.
def detect_objects(net, img):
    """Run object detection over the input image."""
    # Blob dimension (dim x dim)
    dim = 300

    mean = (0, 0, 0)
    
    # Create a blob from the image
    blob = cv2.dnn.blobFromImage(img, 1.0, (dim, dim), mean, True)

    # Pass blob to the network
    net.setInput(blob)
    
    # Peform Prediction
    objects = net.forward()
    return objects

# Display single prediction.
def draw_text(im, text, x, y):
    """Draws text label at a given x-y position with a black background."""
    fontface = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.7
    thickness = 1
    
    # Get text size 
    textSize = cv2.getTextSize(text, fontface, font_scale, thickness)
    dim = textSize[0]
    baseline = textSize[1]
            
    # Use text size to create a black rectangle.
    cv2.rectangle(im, (x, y), (x + dim[0], y + dim[1] + baseline), (0, 0, 0), cv2.FILLED);
    # Display text inside the rectangle.
    cv2.putText(im, text, (x, y + dim[1]), fontface, font_scale, (0, 255, 255), thickness, cv2.LINE_AA)

# Display all predictions.
def draw_objects(im, objects, threshold = 0.25):
    """Displays a box and text for each detected object exceeding the confidence threshold."""
    rows = im.shape[0]
    cols = im.shape[1]

    # For every detected object.
    for i in range(objects.shape[2]):
        # Find the class and confidence.
        classId = int(objects[0, 0, i, 1])
        score = float(objects[0, 0, i, 2])
        
        # Recover original cordinates from normalized coordinates
        x = int(objects[0, 0, i, 3] * cols)
        y = int(objects[0, 0, i, 4] * rows)
        w = int(objects[0, 0, i, 5] * cols - x)
        h = int(objects[0, 0, i, 6] * rows - y)
        # Check if the detection is of good quality
        if score > threshold:
            draw_text(im, "{}".format(labels[classId]), x, y)
            cv2.rectangle(im, (x, y), (x + w, y + h), (255, 255, 255), 2)
    
    return im

class Detector(Node):


    def __init__(self):
        super().__init__('detector_py')

        # Publisher of detection image
        self.detectionImagePublisher_ = self.create_publisher(
            Image,
            "detection_image",
            10)
        self.detectionImagePublisher_
        self.get_logger().info(f"Detection Image publisher topic name {self.detectionImagePublisher_.topic_name}")

        # Publisher of tracked object
        self.tracksPublisher_ = self.create_publisher(
            TrackedObjectsStamped,
            "tracks",
            10)
        self.tracksPublisher_
        self.get_logger().info(f"Tracks publisher topic name {self.tracksPublisher_.topic_name}")

        # Subscriber for recieve AdultUpadets msg
        self.imageSubscriber_ = self.create_subscription(
            Image,
            'image',
            self.camera_callback,
            10)
        self.imageSubscriber_       # prevent unused variable warning
        self.get_logger().info(f"Image subscription topic name {self.imageSubscriber_.topic_name}")


    def camera_callback(self, image_):
        if image_:

            global contimg
            contimg += 1

            print ("Got image"+str(contimg))
            filename='/home/isti/Pictures/output' + str(contimg) + '.jpg'

            # ROS data to publish
            l_detection_image = Image()
            l_tracks = TrackedObjectsStamped()

            if contimg<200:
                bridge = CvBridge()
                cv_image = bridge.imgmsg_to_cv2(image_, desired_encoding="bgr8")
                #magenta = (255,0,255)
                #cv_image = cv2.rectangle(cv_image,  (0,0), (50,50), magenta, thickness = 3, linetype = cv2.LINE_8)
                
                objects = detect_objects(net, cv_image)

		# Each detected object returns a list with the structure of:
		# [[[..., classId, score, x, y, w, h]]]
		#print('Detected {len(objects[0][0])} objects (no confidence filtering)')
                first_detected_obj = objects[0][0][0]
		#print('First object:', first_detected_obj)

                result = draw_objects(cv_image.copy(), objects, 0.4)
		# Save the image to the file system.

                cv2.imwrite(filename, result)


            if self.detectionImagePublisher_.get_subscription_count() > 0:
                self.detectionImagePublisher_.publish(l_detection_image)

            if self.tracksPublisher_.get_subscription_count() > 0:
                self.tracksPublisher_.publish(l_tracks)


