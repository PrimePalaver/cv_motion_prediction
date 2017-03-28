#!/usr/bin/env python

""" Detect a ball in a specified ROS image stream topic. """

import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3


class Detector(object):

    def __init__(self, image_topic):
        """ Initialize the ball detector """

        # Initialize ROS node
        rospy.init_node('ball_detector')

        # Initialize CvBridge
        self.bridge = CvBridge() # used to convert ROS messages to OpenCV

        # Initialize CV images
        self.bgr_image = None # the latest image from the camera
        self.hsv_image = None
        self.hsv_image_blurred = None
        self.binary_image = None

        # Make sure CV images aren't overwritten before they are displayed
        self.curr_images_already_displayed = True

        # Define parameters based on OpenCV version
        if cv2.__version__=='3.1.0-dev':
            self.bgr2gray = cv2.COLOR_BGR2GRAY
        else:
            self.bgr2gray = cv2.cv.CV_BGR2GRAY

        # Create windows
        cv2.namedWindow('bgr_window') # window for unprocessed image
        cv2.namedWindow('binary_window') # window for binary image
        cv2.namedWindow('sliders_window') # window for parameter sliders

        # (0,20,128), (20,255,255) # orange
        # (45, 50, 50), (100, 255, 130) # blue

        # HSV filter sliders
        self.hsv_lb = np.array([0, 20, 128]) # hsv lower bound
        self.hsv_ub = np.array([20, 255, 255]) # hsv upper bound
        cv2.createTrackbar('H lb', 'sliders_window', self.hsv_lb[0], 255,
            self.set_h_lb)
        cv2.createTrackbar('S lb', 'sliders_window', self.hsv_lb[1], 255,
            self.set_s_lb)
        cv2.createTrackbar('V lb', 'sliders_window', self.hsv_lb[2], 255,
            self.set_v_lb)
        cv2.createTrackbar('H ub', 'sliders_window', self.hsv_ub[0], 255,
            self.set_h_ub)
        cv2.createTrackbar('S ub', 'sliders_window', self.hsv_ub[1], 255,
            self.set_s_ub)
        cv2.createTrackbar('V ub', 'sliders_window', self.hsv_ub[2], 255,
            self.set_v_ub)

        # Blur amount slider
        self.blur_amount = 11
        cv2.createTrackbar('blur amount', 'sliders_window', self.blur_amount,
            50, self.set_blur)

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


    def set_blur(self, val):
        """ Slider callback to set blur amount for image processing """

        self.blur_amount = val


    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called bgr_image for subsequent processing """

        print "process_image"

        # Check that we don't overwrite an image that hasn't been displayed yet
        if self.curr_images_already_displayed:
            self.bgr_image = self.bridge.imgmsg_to_cv2(msg,
                desired_encoding="bgr8")
            self.grayscale_image = cv2.cvtColor(self.bgr_image, self.bgr2gray)
            self.hsv_image = cv2.cvtColor(self.bgr_image, cv2.COLOR_BGR2HSV)
            self.blurred_image = cv2.GaussianBlur(self.hsv_image,
                (self.blur_amount, self.blur_amount), 0)

            self.binary_image = cv2.inRange(self.blurred_image, self.hsv_lb,
                self.hsv_ub)
            self.binary_image = cv2.erode(self.binary_image, None, iterations=3)
            self.binary_image = cv2.dilate(self.binary_image, None, iterations=2)

            # blob detection
            contours = cv2.findContours(self.binary_image.copy(),
                cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None

            if len(contours) > 0:
                largest_contour = max(contours, key=cv2.contourArea)
                self.detected_ball = cv2.minEnclosingCircle(largest_contour)
                ((x, y), radius) = self.detected_ball
                moment = cv2.moments(largest_contour)
                center = (int(moment["m10"] / moment["m00"]),
                    int(moment["m01"] / moment["m00"]))

                # drawing blobs
                if radius > 10:
                    cv2.circle(self.bgr_image, (int(x), int(y)), int(radius),
                        (0, 255, 255), 2)
                    cv2.circle(self.bgr_image, center, 5, (0, 0, 255), -1)
                else:
                    print "Radius too small!"

            self.curr_images_already_displayed = False


if __name__ == '__main__':
    node = Detector("/camera/image_rect_color")

    r = rospy.Rate(30)
    
    while not rospy.is_shutdown():
        if (not self.bgr_image is None) and (not self.binary_image is None):
            cv2.imshow('bgr_window', self.bgr_image)
            cv2.imshow('binary_window', self.binary_image)
            cv2.waitKey(5)
            self.curr_images_already_displayed = True

        try:
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            print "Time went backwards. Carry on."
