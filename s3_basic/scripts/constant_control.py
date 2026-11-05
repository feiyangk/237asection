#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class ConstantControl(Node):
    def __init__(self) -> None:
        super().__init__("constant_control")

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.timer = self.create_timer(0.2, self.timer_callback)

	    # create subscription with: self.create_subscription(<msg type>, <topic>, <callback>, <qos>)

        self.kill_sub = self.create_subscription(Bool, "/kill", self.kill_callback, 10)

    def timer_callback(self) -> None:
        msg = Twist()
        msg.linear.x = 0.1  
        self.publisher.publish(msg)

    def kill_callback(self, msg: Bool) -> None:
        if msg.data: # whenever you receive a True message from /kill
            self.timer.cancel()
            self.publisher.publish(Twist())              

if __name__ == "__main__":
    rclpy.init()
    node = ConstantControl()
    rclpy.spin(node)
    rclpy.shutdown()



