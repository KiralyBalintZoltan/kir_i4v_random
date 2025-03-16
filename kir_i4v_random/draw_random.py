#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class FlowerDrawer(Node):
    def __init__(self):
        super().__init__('draw_random')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.draw_flower)  # Timer to call draw_flower every 0.1 seconds
        self.angle = 0.0
        self.radius = 0.5
        self.x = 0.0
        self.y = 0.0
        self.near_wall = False  # Flag to track if the turtle is near a wall
        self.target_distance = 2.0  # Distance to move away from the wall

    def pose_callback(self, pose):
        # Update the turtle's position
        self.x = pose.x
        self.y = pose.y

    def draw_flower(self):
        if not self.near_wall:
            # Continue drawing the flower
            self.angle += 5.0  # Increment the angle for the spiral
            self.radius += 0.05  # Increment the radius for the spiral
        self.move_turtle()

    def move_turtle(self):
        twist = Twist()

        # Check if the turtle is near a wall (within target_distance units)
        if self.x >= 11.0 - self.target_distance or self.x <= self.target_distance or self.y >= 11.0 - self.target_distance or self.y <= self.target_distance:
            self.near_wall = True
            twist.linear.x = 0.0  # Stop moving
            twist.angular.z = 0.0  # Stop rotating
        else:
            if self.near_wall:
                # If the turtle was near a wall but is now safe, resume drawing
                self.near_wall = False
            # Continue drawing the flower
            twist.linear.x = self.radius
            twist.angular.z = self.angle

        # Publish the velocity command
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FlowerDrawer()
    rclpy.spin(node)  # Keep the program running indefinitely
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()