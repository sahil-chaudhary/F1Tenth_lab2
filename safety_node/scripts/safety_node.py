#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        #Create a Publisher to send commands to the car for emergency braking
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        #Create a Subscriber to subscribe /scan topic to recieve the laser scan data
        #Also, use the scan_callback method to process the data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        #Subscribe to the /ego_racecar/odom topic to get the current speed of the vehicle
        #Also, use the odom_callback method to process the data
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        #Initialize the speed of the vehicle to zero
        self.speed = 0.0
        
        #Define the threshold for minimum iTTC(Time to collision)
        self.min_ttc = 0.5

    def odom_callback(self, odom_msg):
        """The function updates the current speed of the car based on the odom message received"""
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        """This function calculates the time to collision(TTC) based on the laser scan data and triggers emergency braking if TTC is less than the threshold"""

        #Get the range data from the laser scan
        ranges = scan_msg.ranges

        #Get the minimum range from the laser scan data
        min_range = min(ranges)

        #Calculate the time to collision based on the minimum range and current speed of the vehicle
        ttc = min_range / self.speed

        #If the time to collision is less than the threshold, trigger emergency braking
        if ttc < self.min_ttc:
            self.get_logger().info("Emergency Braking Triggered")
            self.emergency_brake()

    def emergency_brake(self):
        """This function sends the command to the car to stop the vehicle immediately"""
        #Create an AckermannDriveStamped message to stop the vehicle
        stop_msg = AckermannDriveStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        stop_msg.drive.steering_angle = 0.0
        stop_msg.drive.speed = 0.0

        #Publish the stop message to the drive topic
        self.drive_pub.publish(stop_msg)
        
def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()