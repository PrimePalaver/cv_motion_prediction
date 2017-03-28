#!/usr/bin/env python

from detector import Detector

class Main(object):

    def __init__(self):
        self.detector = Detector("/camera/image_rect_color")
        self.detector.run()


    def run(self):
        """ The main run loop"""
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Create windows to show all specified image streams
            if (not self.bgr_image is None):
                cv2.imshow('video_window', self.bgr_image)
            if (not self.filt_image is None):
                cv2.imshow('filt_window', self.filt_image)
            if (not self.cropped_sign_grayscale is None):
                cv2.imshow('cropped_grayscale_window', self.cropped_sign_grayscale)
            if (not self.template_matcher.img_T is None):
                cv2.imshow('img_T_window', self.template_matcher.img_T)
            cv2.waitKey(5)
            r.sleep()


if __name__ == '__main__':
    node = Main()
    #node.run()
