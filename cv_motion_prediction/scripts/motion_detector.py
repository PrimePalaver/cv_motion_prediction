#!/usr/bin/env python

""" Detect a ball, calculate its 3D position, and predict its future behavior. """

import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3


class MotionDetector(object):

    def __init__(self, image_topic):
        """ Initialize the ball tracker """

        # Initialize ROS node
        rospy.init_node('motion_detector')

        # Initialize CvBridge
        self.bridge = CvBridge() # used to convert ROS messages to OpenCV

        # Initialize CV images
        self.bgr_image = None # the latest image from the camera
        self.hsv_image = None
        self.hsv_image_blurred = None
        self.binary_image = None

        if cv2.__version__=='3.1.0-dev':
            self.bgr2gray = cv2.COLOR_BGR2GRAY
            self.hough_gradient = cv2.HOUGH_GRADIENT
        else:
            self.bgr2gray = cv2.cv.CV_BGR2GRAY
            self.hough_gradient = cv2.cv.CV_HOUGH_GRADIENT

        # Create windows
        cv2.namedWindow('bgr_window') # window for unprocessed image
        cv2.namedWindow('binary_window') # window for binary image
        cv2.namedWindow('sliders_window') # window for parameter sliders

        # HSV filter sliders
        self.hsv_lb = np.array([0, 90, 210]) # hsv lower bound
        cv2.createTrackbar('H lb', 'sliders_window', self.hsv_lb[0], 255,
            self.set_h_lb)
        cv2.createTrackbar('S lb', 'sliders_window', self.hsv_lb[1], 255,
            self.set_s_lb)
        cv2.createTrackbar('V lb', 'sliders_window', self.hsv_lb[2], 255,
            self.set_v_lb)
        self.hsv_ub = np.array([20, 230, 255]) # hsv upper bound
        cv2.createTrackbar('H ub', 'sliders_window', self.hsv_ub[0], 255,
            self.set_h_ub)
        cv2.createTrackbar('S ub', 'sliders_window', self.hsv_ub[1], 255,
            self.set_s_ub)
        cv2.createTrackbar('V ub', 'sliders_window', self.hsv_ub[2], 255,
            self.set_v_ub)

        # Circle detection parameters sliders
        self.circle_params = np.array([140, 85, 85])
        cv2.createTrackbar('dp', 'sliders_window', self.circle_params[0],
            200, self.set_dp)
        cv2.createTrackbar('param1', 'sliders_window', self.circle_params[1],
            200, self.set_param1)
        cv2.createTrackbar('param2', 'sliders_window', self.circle_params[2],
            200, self.set_param2)

        # Blur amount slider
        self.blur_amount = 1
        cv2.createTrackbar('blur amount', 'sliders_window', self.blur_amount,
            51, self.set_blur)

        # Suscribe to ROS camera feed from Neato robot
        rospy.Subscriber(image_topic, Image, self.process_image)

        print "initialization complete"


    def set_h_lb(self, val):
        """ Slider callback to set hue lower bound """

        self.hsv_lb[0] = val


    def set_s_lb(self, val):
        """ Slider callback to set saturation lower bound """

        self.hsv_lb[1] = val


    def set_v_lb(self, val):
        """ Slider callback to set value lower bound """

        self.hsv_lb[2] = val


    def set_h_ub(self, val):
        """ Slider callback to set hue upper bound """

        self.hsv_ub[0] = val


    def set_s_ub(self, val):
        """ Slider callback to set saturation upper bound """

        self.hsv_ub[1] = val


    def set_v_ub(self, val):
        """ Slider callback to set value upper bound """

        self.hsv_ub[2] = val


    def set_dp(self, val):
        """ Slider callback to set dp for circle detection """

        self.circle_params[0] = val


    def set_param1(self, val):
        """ Slider callback to set param1 for circle detection """

        self.circle_params[1] = val


    def set_param2(self, val):
        """ Slider callback to set param2 for circle detection """

        self.circle_params[2] = val


    def set_blur(self, val):
        """ Slider callback to set blur amount for image processing """

        self.blur_amount = 2*val+1


    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called bgr_image for subsequent processing """

        print "process_image"

        self.bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.grayscale_image = cv2.cvtColor(self.bgr_image, self.bgr2gray)
        self.hsv_image = cv2.cvtColor(self.bgr_image, cv2.COLOR_BGR2HSV)
        self.hsv_image = cv2.medianBlur(self.hsv_image, 5)
        self.binary_image = cv2.inRange(self.hsv_image, self.hsv_lb, self.hsv_ub)

        # Parameters for cv2.HoughCircles
        # dp: inverse ratio of the resolution (smaller = detect less circular
        #     circles)
        # minDist: minimum distance between the center of detected circles
        # param1: gradient value used for edge detection
        # param2: threshold for center detection
        # minRadius: minimum size of circle radius in pixels
        # maxRadius: maximum size of circle radius in pixels
        circles = cv2.HoughCircles(self.grayscale_image,
            self.hough_gradient, minDist=30,
            dp=self.circle_params[0]/float(100),
            param1=self.circle_params[1], param2=self.circle_params[2],
            minRadius=0, maxRadius=0)
        
        if (circles != None and len(circles)):
            circles = np.uint16(np.around(circles))
        
            for i in circles[0,:]:
                # Draw outer circle
                cv2.circle(self.bgr_image, (i[0], i[1]), i[2], (0,255,0), 2)
                # Draw circle center
                cv2.circle(self.bgr_image, (i[0], i[1]), 2, (255,0,0), 3)
                print "circles:", i[0], i[1]
                
        else:
            print "no circles!"


    def run(self):
        """  """
        r = rospy.Rate(30)
        
        while not rospy.is_shutdown():

            if not self.bgr_image is None:
                cv2.imshow('video_window', self.bgr_image)
                cv2.waitKey(5)

            if not self.binary_image is None:                
                cv2.imshow('binary_window', self.binary_image)
                cv2.waitKey(5)

            r.sleep()


if __name__ == '__main__':
    node = MotionDetector("/camera/image_raw")
    node.run()
