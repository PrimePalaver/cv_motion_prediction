#!/usr/bin/env python

""" Predicts the future 3D position of the ball """

import rospy
import math
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PointStamped, Point, Pose, Vector3
from std_msgs.msg import Header,ColorRGBA
from visualization_msgs.msg import Marker

class Predictor(object):

    def __init__(self):

        # Initialize ROS node
        rospy.init_node('predictor')

        #Subscribe to 3D position of detected ball
        self.sub = rospy.Subscriber('ball_3d_position', PointStamped, self.estimate_velocity)

        #Create publisher for estimated velocity vector
        self.pub = rospy.Publisher('ball_velocity', Marker, queue_size=10)

        self.previous_position = PointStamped(header=Header(stamp=rospy.Time.now(), frame_id="base_link"))


    def estimate_velocity(self, msg):
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        x_diff = x - self.previous_position.point.x
        y_diff = y - self.previous_position.point.y
        z_diff = z - self.previous_position.point.z
        t_diff = (msg.header.stamp.secs+msg.header.stamp.nsecs/10.0**9) - (self.previous_position.header.stamp.secs + self.previous_position.header.stamp.nsecs/10.0**9)

        x_vel = x_diff/t_diff
        y_vel = y_diff/t_diff
        z_vel = z_diff/t_diff

        scale = 1.0
        vel = Marker()
        vel.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        vel.type = Marker.ARROW
        vel.points = [Point(x=x,y=y, z=z),
                      Point(x=x+scale*x_diff, y=y+scale*y_diff, z=z+scale*z_diff)]
        vel.color = ColorRGBA(r=255, g=0, b=0, a=1.0)
        vel.scale = Vector3(x=.05, y=.05)

        self.pub.publish(vel)

    def run(self):
        """ Main run function """
        
        r = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                print "Time went backwards. Carry on."

if __name__ == '__main__':
    predictor = Predictor()
    predictor.run()