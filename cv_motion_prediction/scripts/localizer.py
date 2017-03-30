#!/usr/bin/env python

""" Calculates the 3D position of the ball given the ball's physical
    radius (m), image radius (pixels), image x position (pixels), and 
    image y position (pixels). """

import math
from cv_motion_prediction.msg import Circle
from geometry_msgs.msg import Point

class Localizer(object):

    def __init__(self):

        # Initialize ROS node
        rospy.init_node('localizer')

        # Suscribe to position of detected ball in image
        self.sub = rospy.Subscriber('detected_ball', Circle, self.get_3d_position)

        # Create publisher for current detected ball characteristics
        self.pub = rospy.Publisher('ball_3d_position', Point, queue_size=10)

        # Set member variables
        self.physical_circumference = .67 # for big blue ball
        self.physical_radius = self.physical_circumference / float(2 * math.pi)
        self.x_optical_center = 319.5
        self.y_optical_center = 239.5
        # TODO: get the focal lengths from the camera calibration file
        self.x_focal_length = 331.703
        self.y_focal_length = 331.267
        self.avg_focal_length = (self.x_focal_length +
            self.y_focal_length) / float(2)


    def get_3d_position(self, msg):
        x_pos = msg.x
        y_pos = msg.y
        image_radius = msg.radius

        x_cam = x_pos - self.x_optical_center
        y_cam = 480 - y_pos - self.y_optical_center

        print "Cams: ", x_cam, y_cam

        m_x = x_cam / self.x_focal_length
        m_y = y_cam / self.y_focal_length
 
        print "Slopes: ", m_x, m_y       

        d = (self.avg_focal_length * self.physical_radius) / float(image_radius)

        print "D: ", d

        x = d * math.sin(math.atan2(x_cam, self.x_focal_length))
        y = d * math.sin(math.atan2(y_cam, self.y_focal_length))
        z = d * math.cos(math.atan2(x_cam, self.x_focal_length))

        point = Point(x, y, z)
        self.pub.publish(x, y, z)


    def run(self):
        """ Main run function """
        
        r = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                print "Time went backwards. Carry on."


if __name__ == '__main__':
    localizer = Localizer()
    localizer.run()
