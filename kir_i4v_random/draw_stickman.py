#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class StickmanDrawer(Node):
    def __init__(self):
        super().__init__('stickman_drawer')
        # Create a publisher to send velocity commands to the turtle
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.draw_stickman)
        self.step = 0
        self.drawing_steps = [
            # Head (circle)
            ("circle", 1.0, 0.0, 2 * math.pi),  # Draw a circle for the head
            # Body (line)
            ("line", 0.0, -1.5, 0.0),  # Move down to draw the body
            # Left arm (line)
            ("line", -1.5, 1.0, 0.0),  # Move to the left for the left arm
            #Move
            ("line", 1.5, -1.0, 0.0),   # Move to the right for the right arm
            # Right arm (line)
            ("line", 0.0, 1.0, 1.5),   # Move to the right for the right arm
            #Move2
            ("line", 1.5, -1.0, 0.0),   # Move to the right for the right arm
             # Body (line)
            ("line", 0.0, -1.0, 0.0),  # Move down to draw the body
            # Left leg (line)
            ("line", 1.5, -1.0, 0.0),  # Move to the left for the left leg
            #Move2
            ("line", -1.5, 1.0, 0.0),   # Move to the right for the right arm
            # Right leg (line)
            ("line", 1.5, -1.0, 0.0),   # Move to the right for the right leg
        ]
        self.current_step = 0

    def draw_stickman(self):
        if self.current_step >= len(self.drawing_steps):
            # Stop the turtle after completing the stickman
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            self.get_logger().info("Stickman drawing complete!")
            self.timer.cancel()
            return

        action, x, y, angle = self.drawing_steps[self.current_step]
        twist = Twist()

        if action == "circle":
            # Draw a circle for the head
            twist.linear.x = 1.0  # Move forward
            twist.angular.z = 1.0  # Rotate in place
            if self.step >= int(2 * math.pi / 0.1):  # Complete the circle
                self.current_step += 1
                self.step = 0
        elif action == "line":
            # Move in a straight line
            twist.linear.x = x
            twist.linear.y = y
            twist.angular.z = angle
            if self.step >= 10:  # Adjust steps for line length
                self.current_step += 1
                self.step = 0

        # Publish the velocity command
        self.publisher.publish(twist)
        self.step += 1

def main(args=None):
    rclpy.init(args=args)
    stickman_drawer = StickmanDrawer()
    rclpy.spin(stickman_drawer)
    stickman_drawer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()