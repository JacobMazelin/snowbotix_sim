#!/usr/bin/env python3
"""
Simple vehicle controller node for Snowbotix Sim.

This node demonstrates the development workflow:
1. Edit this file in VS Code
2. Save (Cmd+S)
3. Commit and push (Cmd+Shift+P → "Git: Push")
4. Wait 30-60 seconds
5. See changes in Foxglove!
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class VehicleController(Node):
    """
    A simple controller that moves a vehicle in a circle.
    
    You can modify the parameters below to change the behavior:
    - vehicle_name: Which vehicle to control (vehicle_blue or vehicle_green)
    - linear_velocity: Speed in m/s
    - angular_velocity: Rotation rate in rad/s
    """
    
    def __init__(self):
        super().__init__('vehicle_controller')
        
        # Parameters you can modify!
        self.declare_parameter('vehicle_name', 'vehicle_blue')
        self.declare_parameter('linear_velocity', 1.0)  # m/s
        self.declare_parameter('angular_velocity', 0.5)  # rad/s
        
        self.vehicle_name = self.get_parameter('vehicle_name').value
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.angular_velocity = self.get_parameter('angular_velocity').value
        
        # Create publisher
        topic_name = f'/model/{self.vehicle_name}/cmd_vel'
        self.publisher = self.create_publisher(Twist, topic_name, 10)
        
        # Create timer to publish commands at 10 Hz
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(
            f'Controller started for {self.vehicle_name}\n'
            f'  Linear velocity: {self.linear_velocity} m/s\n'
            f'  Angular velocity: {self.angular_velocity} rad/s\n'
            f'  Publishing to: {topic_name}'
        )
        
        self.counter = 0
    
    def timer_callback(self):
        """Publish velocity command to make vehicle move in a circle."""
        msg = Twist()
        
        # Set linear and angular velocities
        msg.linear.x = self.linear_velocity
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular_velocity
        
        # Publish the message
        self.publisher.publish(msg)
        
        # Log every 50 messages (every 5 seconds)
        self.counter += 1
        if self.counter % 50 == 0:
            self.get_logger().info(
                f'Moving {self.vehicle_name}: '
                f'linear={self.linear_velocity:.2f}, '
                f'angular={self.angular_velocity:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = VehicleController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
