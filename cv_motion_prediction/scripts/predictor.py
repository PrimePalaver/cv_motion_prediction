#!/usr/bin/env python

""" Calculates the velocity of a ball given a stream of the ball's
    position in 3D space. """

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
        self.sub = rospy.Subscriber('ball_3d_position', PointStamped,
            self.estimate_velocity)

        #Create publisher for estimated velocity vector
        self.pub = rospy.Publisher('ball_velocity', Marker, queue_size=10)

        # Initialize the list for the rolling average in `estimate_velocity`
        self.previous_position = PointStamped(
            header=Header(stamp=rospy.Time.now(), frame_id="base_link"))
        self.previous_velocities = [(0.0, 0.0, 0.0)] * 10


    def estimate_velocity(self, msg):
        """ Publish a Marker that represents the current velocity of a
            ball in 3D space, given a stream of the ball's position. Use
            a rolling average to smooth the output. """

        # Define the ball's current position
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        # Define the difference between the ball's current position and the
        # ball's position at the last time step
        x_diff = x - self.previous_position.point.x
        y_diff = y - self.previous_position.point.y
        z_diff = z - self.previous_position.point.z
        t_diff = (msg.header.stamp.secs+msg.header.stamp.nsecs/10.0**9) - \
            (self.previous_position.header.stamp.secs +
            self.previous_position.header.stamp.nsecs/10.0**9)

        # Define the ball's instantaneous velocity
        x_vel = x_diff/t_diff
        y_vel = y_diff/t_diff
        z_vel = z_diff/t_diff

        # Update the array of velocities used for the rolling average
        self.previous_position = msg
        self.previous_velocities.append((x_vel, y_vel, z_vel))
        self.previous_velocities = self.previous_velocities[1:]

        # Calculate the rolling average of all velocities
        vel_avg = [float(sum(axis)) / len(axis) for axis in
            zip(*self.previous_velocities)]

        # Create a Marker with the average velocity
        scale = 1.0
        vel = Marker()
        vel.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        vel.type = Marker.ARROW
        vel.points = [Point(x=x,y=y, z=z), Point(x=x+scale*vel_avg[0],
            y=y+scale*vel_avg[1], z=z+scale*vel_avg[2])]
        vel.color = ColorRGBA(r=255, g=0, b=0, a=1.0)
        vel.scale = Vector3(x=.05, y=.05)

        # Publish the Marker
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
