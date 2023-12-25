#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point
import math

class MoveToCoordinates:
    def __init__(self, target_coordinates):
        self.target_coordinates = target_coordinates
        self.current_pose = Pose()
        self.target_index = 0
        self.move_to_next_target = True

        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.pose_callback)#replace with whatever topic gives u robots current path
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)#replace with whatever topic manipulates robots velocities

    def pose_callback(self, odom):
        rospy.loginfo(self.current_pose)
        self.current_pose.position.x = odom.pose.pose.position.x
        self.current_pose.position.y = odom.pose.pose.position.y
        self.current_pose.orientation.z = math.atan2(
            2.0 * (odom.pose.pose.orientation.w * odom.pose.pose.orientation.z + odom.pose.pose.orientation.x * odom.pose.pose.orientation.y),
            1.0 - 2.0 * (odom.pose.pose.orientation.y**2 + odom.pose.pose.orientation.z**2)
        )

        if self.move_to_next_target:
            self.move_turtle_to_next_target()

    def move_turtle_to_next_target(self):
        if self.target_index < len(self.target_coordinates):
            target_coordinate = self.target_coordinates[self.target_index]
            distance_to_target = math.sqrt((target_coordinate.x - self.current_pose.position.x)**2 + (target_coordinate.y - self.current_pose.position.y)**2)

            if distance_to_target > 0.1:
                # Move towards the target coordinate
                self.rotate_towards_target(target_coordinate)
            else:
                # Move to the next target if available
                self.target_index += 1 
                if self.target_index < len(self.target_coordinates):
                    rospy.loginfo('Moving to the next target: {}'.format(self.target_coordinates[self.target_index]))
                else:
                    rospy.loginfo('Reached the last target. Resting in place.')

    def rotate_towards_target(self, target_coordinate):
        target_theta = math.atan2(target_coordinate.y - self.current_pose.position.y, target_coordinate.x - self.current_pose.position.x)
        angular_velocity = 0.25  # Adjust the angular velocity as needed

        if abs(self.current_pose.orientation.z - target_theta) > 0.01:
            # Rotate until the turtle points towards the target coordinate
            twist_msg = Twist()
            twist_msg.angular.z = angular_velocity if target_theta > self.current_pose.orientation.z else -angular_velocity
            self.cmd_vel_publisher.publish(twist_msg)
        else:
            # Stop rotating when the turtle points towards the target
            twist_msg = Twist()
            self.cmd_vel_publisher.publish(twist_msg)
            self.move_forward_to_target(target_coordinate)

    def move_forward_to_target(self, target_coordinate):
        linear_velocity = 0.25  # Adjust the linear velocity as needed

        distance_to_target = math.sqrt((target_coordinate.x - self.current_pose.position.x)**2 + (target_coordinate.y - self.current_pose.position.y)**2)
        if distance_to_target > 0.1:
            # Move forward until the turtle reaches the target coordinate
            twist_msg = Twist()
            twist_msg.linear.x = linear_velocity
            self.cmd_vel_publisher.publish(twist_msg)
        else:
            # Stop moving when the turtle reaches the target
            twist_msg = Twist()
            self.cmd_vel_publisher.publish(twist_msg)
            self.move_to_next_target = True


if __name__ == '__main__':
    try:
        rospy.init_node('move_to_coordinates_node')
        
        # Define a list of Point coordinates
        target_coordinates = [Point(x=3.5, y=1.5)]#u can add whatever points u want the robot to visit in this

        move_to_coordinates = MoveToCoordinates(target_coordinates)
        rospy.loginfo('Moving to the first target: {}'.format(target_coordinates[0]))
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
