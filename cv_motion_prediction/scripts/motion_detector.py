#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

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
        rospy.init_node('motion_detector')
        self.cv_image = None                        # the latest image from the camera
        self.hsv_image = None
        self.threshold_image = None
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        self.image_info_window = None
        self.hsv_image_blurred = None

        cv2.namedWindow('video_window')
        cv2.setMouseCallback('video_window', self.process_mouse_event)

        # rospy.on_shutdown(self.stop)    

        # hsv slider
        cv2.namedWindow('threshold_image')
        self.hsv_lb = np.array([0, 90, 210]) # hsv lower bound
        cv2.createTrackbar('H lb', 'threshold_image', self.hsv_lb[0], 255, self.set_h_lb)
        cv2.createTrackbar('S lb', 'threshold_image', self.hsv_lb[1], 255, self.set_s_lb)
        cv2.createTrackbar('V lb', 'threshold_image', self.hsv_lb[2], 255, self.set_v_lb)
        self.hsv_ub = np.array([20, 230, 255]) # hsv upper bound
        cv2.createTrackbar('H ub', 'threshold_image', self.hsv_ub[0], 255, self.set_h_ub)
        cv2.createTrackbar('S ub', 'threshold_image', self.hsv_ub[1], 255, self.set_s_ub)
        cv2.createTrackbar('V ub', 'threshold_image', self.hsv_ub[2], 255, self.set_v_ub)

        # circle detection slider
        self.circle_params = np.array([85, 85])
        cv2.createTrackbar('param1', 'threshold_image', self.circle_params[0], 200, self.set_param1)
        cv2.createTrackbar('param2', 'threshold_image', self.circle_params[1], 200, self.set_param2)

        # blur amount slider
        self.blur_amount = 1
        cv2.createTrackbar('blur amount', 'threshold_image', self.blur_amount, 51, self.set_blur)

        rospy.Subscriber(image_topic, Image, self.process_image)

        print "initialization complete"

    # hsv slider callback functions
    def set_h_lb(self, val):
        """ set hue lower bound """
        self.hsv_lb[0] = val

    def set_s_lb(self, val):
        """ set saturation lower bound """
        self.hsv_lb[1] = val

    def set_v_lb(self, val):
        """ set value lower bound """
        self.hsv_lb[2] = val

    def set_h_ub(self, val):
        """ set hue upper bound """
        self.hsv_ub[0] = val

    def set_s_ub(self, val):
        """ set saturation upper bound """
        self.hsv_ub[1] = val

    def set_v_ub(self, val):
        """ set value upper bound """
        self.hsv_ub[2] = val

    def set_param1(self, val):
        """ set param1 for circle detection """
        self.circle_params[0] = val

    def set_param2(self, val):
        """ set param2 for circle detection """
        self.circle_params[1] = val

    def set_blur(self, val):
        """ set blur amount for HSV image"""
        self.blur_amount = 2*val+1

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """

        print "process_image"

        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        self.hsv_image = cv2.medianBlur(self.hsv_image, 5)
        #cv2.GaussianBlur(self.hsv_image, self.hsv_image_blurred, )
        self.grayscale_image = cv2.cvtColor(self.cv_image, cv2.cv.CV_BGR2GRAY)

        self.threshold_image = cv2.inRange(self.hsv_image, self.hsv_lb, self.hsv_ub)
        self.threshold_image = cv2.medianBlur(self.threshold_image, self.blur_amount)

        # dp: inverse ratio of the resolution (smaller = detect less circular
        #     circles)
        # minDist: minimum distance between the center of detected circles
        # param1: gradient value used for edge detection
        # param2: threshold for center detection
        # minRadius: minimum size of circle radius in pixels
        # maxRadius: maximum size of circle radius in pixels

        if cv2.__version__=='3.1.0-dev':
            hough_gradient = cv2.HOUGH_GRADIENT
        else:
            hough_gradient = cv2.cv.CV_HOUGH_GRADIENT

        circles = cv2.HoughCircles(self.grayscale_image, cv2.cv.CV_HOUGH_GRADIENT, dp=2, minDist=30, param1=self.circle_params[0], param2=self.circle_params[1], minRadius=0, maxRadius=0)
        
        if (circles != None and len(circles)):
            circles = np.uint16(np.around(circles))
        
            for i in circles[0,:]:
                # draw outer circle
                cv2.circle(self.cv_image, (i[0], i[1]), i[2], (0,255,0), 2)
                # draw circle center
                cv2.circle(self.cv_image, (i[0], i[1]), 2, (255,0,0), 3)
                print "circles:", i[0], i[1]
        else:
            print "no circles!"

        
        cv2.imshow('video_window', self.cv_image)
        cv2.waitKey(5)



    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values associated
        with a particular pixel in the camera images """
        self.image_info_window = 255*np.ones((500,500,3))

        # show hsv values
        cv2.putText(self.image_info_window,
                    'Color (h=%d,s=%d,v=%d)' % (self.hsv_image[y,x,0], self.hsv_image[y,x,1], self.hsv_image[y,x,2]),
                    (5,50), # 5 = x, 50 = y
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))

    # def stop(self):
    #     twist = Twist()
    #     self.pub.publish(twist)

    def run(self):
        r = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            if not self.threshold_image is None:                
                cv2.imshow('threshold_image', self.threshold_image)
                cv2.waitKey(5)

            if not self.image_info_window is None:
                cv2.imshow('image_info', self.image_info_window)
                cv2.waitKey(5)

            r.sleep()

if __name__ == '__main__':
    node = MotionDetector("/camera/image_rect_color")
    node.run()
