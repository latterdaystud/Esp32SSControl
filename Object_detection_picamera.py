######## Autonomous Laser Turret #########
#
# Author: Kyle Koehn
# Date: 4/4/19
# Description:
# This code have been modified from Even Juras's Picamera Object Detection Using Tensorflow Classifier.
# Using TensorFlow, this program will use a PiCamera to detect objects and decide if they are a target.
# If so, the coordinates of the target will be sent to the servos in order to move the camera and laser.
# Once the laser is on target and is in the center of the screen, a command will be sent to activate the laser.
#
# This is the link to to a tutorial on how to install TensorFlow on a Raspberry Pi.
# https://www.youtube.com/watch?v=npZ-8Nj1YwY&t=540s

## st start with permission on UART sudo chmod 6666 /dev/ttyS0

# Import packages
import os
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import tensorflow as tf
import argparse
import sys
import json
import RPi.GPIO as GPIO
import serial
import json
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)
#GPIO.setup(24, GPIO.OUT)
GPIO.output(23, GPIO.HIGH)
#GPIO.output(24, GPIO.HIGH)
time.sleep(1)
GPIO.output(23, GPIO.LOW)
#GPIO.output(24, GPIO.LOW)

# Set up camera constants
#IM_WIDTH = 1280
#IM_HEIGHT = 720
IM_WIDTH = 640    #Use smaller resolution for
IM_HEIGHT = 480   #slightly faster framerate
#IM_WIDTH = 320
#IM_HEIGHT = 320

SERIAL_PORT = "/dev/ttyS0"
ser = serial.Serial(SERIAL_PORT, baudrate = 9600, timeout = 5)
servoCoor = {}

# Select camera type (if user enters --usbcam when calling this script,
# a USB webcam will be used)
camera_type = 'picamera'
#camera_type = 'usb'
parser = argparse.ArgumentParser()
parser.add_argument('--usbcam', help='Use a USB webcam instead of picamera',
                    action='store_true')
args = parser.parse_args()
if args.usbcam:
    camera_type = 'usb'

# This is needed since the working directory is the object_detection folder.
sys.path.append('..')

# Import utilites
from utils import label_map_util
from utils import visualization_utils as vis_util

# Name of the directory containing the object detection module we're using
MODEL_NAME = 'ssdlite_mobilenet_v2_coco_2018_05_09'

# Grab path to current working directory
CWD_PATH = os.getcwd()

# Path to frozen detection graph .pb file, which contains the model that is used
# for object detection.
PATH_TO_CKPT = os.path.join(CWD_PATH, MODEL_NAME, 'frozen_inference_graph.pb')

# Path to label map file
PATH_TO_LABELS = os.path.join(CWD_PATH, 'data', 'mscoco_label_map.pbtxt')

# Number of classes the object detector can identify
NUM_CLASSES = 90

## Load the label map.
# Label maps map indices to category names, so that when the convolution
# network predicts `5`, we know that this corresponds to `airplane`.
# Here we use internal utility functions, but anything that returns a
# dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,
                                                            use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Load the Tensorflow model into memory.
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    sess = tf.Session(graph=detection_graph)

# Define input and output tensors (i.e. data) for the object detection classifier

# Input tensor is the image
image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

# Output tensors are the detection boxes, scores, and classes
# Each box represents a part of the image where a particular object was detected
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

# Each score represents level of confidence for each of the objects.
# The score is shown on the result image, together with the class label.
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

# Number of objects detected
num_detections = detection_graph.get_tensor_by_name('num_detections:0')

# Initialize frame rate calculation
frame_rate_calc = 1
freq = cv2.getTickFrequency()
font = cv2.FONT_HERSHEY_SIMPLEX

# Initialize camera and perform object detection.
# The camera has to be set up and used differently depending on if it's a
# Picamera or USB webcam.

# I know this is ugly, but I basically copy+pasted the code for the object
# detection loop twice, and made one work for Picamera and the other work
# for USB.

### Picamera ###
if camera_type == 'picamera':
    # Initialize Picamera and grab reference to the raw capture
    camera = PiCamera()
    camera.resolution = (IM_WIDTH, IM_HEIGHT)
    camera.framerate = 20
    rawCapture = PiRGBArray(camera, size=(IM_WIDTH, IM_HEIGHT))
    rawCapture.truncate(0)

    for frame1 in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

        t1 = cv2.getTickCount()

        # Acquire frame and expand frame dimensions to have shape: [1, None, None, 3]
        # i.e. a single-column array, where each item in the column has the pixel RGB value
        frame = np.copy(frame1.array)
        frame.setflags(write=1)
        frame_expanded = np.expand_dims(frame, axis=0)

        # Perform the actual detection by running the model with the image as input
        (boxes, scores, classes, num) = sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: frame_expanded})

        # Draw the results of the detection (aka 'visulaize the results')
        vis_util.visualize_boxes_and_labels_on_image_array(
            frame,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=8,
            min_score_thresh=0.40)

        cv2.putText(frame, "FPS: {0:.2f}".format(frame_rate_calc), (30, 50), font, 1, (255, 255, 0), 2, cv2.LINE_AA)

        # Draw boxes defining "outside" and "inside" locations.
        # Define inside box coordinates (top left and bottom right)
        global inside_counter

        # x and y are coordinates for the target
        pause = 0
        target_flag = 0
        x = 0
        y = 0
        Center = (int(IM_WIDTH * 0.5), int(IM_HEIGHT * 0.5))
        TL_inside = (int(IM_WIDTH * 0.45), int(IM_HEIGHT * 0.45))
        BR_inside = (int(IM_WIDTH * 0.55), int(IM_HEIGHT * 0.55))
        #cv2.circle(frame, TL_inside, 15, (0, 190, 0), 3)
        cv2.rectangle(frame, TL_inside, BR_inside, (255, 20, 20), 3)

        # Check the class of the top detected object by looking at classes[0][0].
        # If the top detected object is a cat (17) or a dog (18) (or a teddy bear (88) for test purposes),
        # find it's center coordinates by looking at the boxes[0][0] variable.
        # boxes[0][0] variable holds coordinates of detected objects as (ymin, xmin, ymax, xmax)
        if (((int(classes[0][0]) == 62) or (int(classes[0][0] == 34) or (int(classes[0][0]) == 51))) and (pause == 0)):
            x = int(((boxes[0][0][1] + boxes[0][0][3]) / 2) * IM_WIDTH)
            y = int(((boxes[0][0][0] + boxes[0][0][2]) / 2) * IM_HEIGHT)
        # check for the second most confident object.
        if (((int(classes[0][1]) == 62) or (int(classes[0][1] == 34) or (int(classes[0][1]) == 51))) and (pause == 0)):
            x = int(((boxes[0][1][1] + boxes[0][1][3]) / 2) * IM_WIDTH)
            y = int(((boxes[0][1][0] + boxes[0][1][2]) / 2) * IM_HEIGHT)

        if (((int(classes[0][2]) == 62) or (int(classes[0][2] == 34) or (int(classes[0][2]) == 51))) and (pause == 0)):
            x = int(((boxes[0][2][1] + boxes[0][2][3]) / 2) * IM_WIDTH)
            y = int(((boxes[0][2][0] + boxes[0][2][2]) / 2) * IM_HEIGHT)

        # If object is in inside box, increment inside counter variable
        if ((x > TL_inside[0]) and (x < BR_inside[0]) and (y > TL_inside[1]) and (y < BR_inside[1])):
            target_flag = 1
            GPIO.output(23, GPIO.HIGH)
        else:
            target_flag = 0
            GPIO.output(23, GPIO.LOW)

        # prepare json for sending
        servoCoor['x'] = x
        servoCoor['y'] = y
        servoCoor['h'] = IM_WIDTH/2
        servoCoor['v'] = IM_HEIGHT/2
        servoCoor['t'] = 25
        servoCoor['p'] = 25

    
        null = "\0"

        jsonData = json.dumps(servoCoor)
        ser.write(null.encode())
        ser.write(jsonData.encode())
        ser.write(null.encode())

        # Draw a circle at center of object
        cv2.circle(frame, (x, y), 5, (75, 13, 180), -1)

        cv2.putText(frame, 'Target Flag: ' + str(target_flag), (10, 150), font, 0.5, (255, 255, 0), 1,
                    cv2.LINE_AA)

        # All the results have been drawn on the frame, so it's time to display it.
        cv2.imshow('Object detector', frame)
        # uncomment to display image
        t2 = cv2.getTickCount()
        time1 = (t2 - t1) / freq
        frame_rate_calc = 1 / time1

        # Press 'q' to quit
        if cv2.waitKey(1) == ord('q'):
            break

        rawCapture.truncate(0)

    camera.close()


cv2.destroyAllWindows()
