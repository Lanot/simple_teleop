#! /usr/bin/env python

import traceback
import atexit
# from threading import Thread
from pynput import keyboard
from pynput.keyboard import Key

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist

class TeleopControl(Node):
    def __init__(self):
        atexit.register(self.publish_stop)

        super().__init__('teleop_control')

        self.declare_parameter('topic', '/cmd_vel')
        self.declare_parameter("step", 0.2)
        self.declare_parameter("publish_rate", 10)

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.step = self.get_parameter("step").value
        self.publish_rate = self.get_parameter("publish_rate").value

        self.vel = Twist()
        self.vel_pub = self.create_publisher(Twist, self.topic, 10)
        self.create_timer(1 / self.publish_rate, self.publish_vel)

        self.print_info()
        self.run()

    def publish_stop(self):
        self.vel_pub.publish(Twist())

    # 20Hz - works good
    def publish_vel(self):
        print("publish_vel: ", self.vel)
        self.vel_pub.publish(self.vel)

    def print_info(self):
        self.get_logger().info(f"\n\tPublishing teleop command to {self.topic}\n\
            Step is {self.step}\n\
            Publish Rate is {self.publish_rate}")

        self.get_logger().info("\n\n\tUse Arrows to control linear velocities:\n\
            ↑ / ↓ - X axis - Move Forward & Backwards\n\
            ← / → - Y axis - Move Left & Right\n\n\
            w/s - Z axis - Move UP & DOWN\n\
            a/d - Z axis - Rotate/YAW Left & Right\n\n\
            esc: QUIT,\n\
            other key: STOP movement")

    def on_press(self, key):
        has_char = hasattr(key, 'char')

        if key == Key.esc:
            print("\n~ ESC ~ key clicked\n")
            raise KeyboardInterrupt()

        # move backward / forward by x-axis
        elif key == Key.up:
            self.vel.linear.x += self.step
        elif key == Key.down:
            self.vel.linear.x -= self.step

        # move backward / forward by y-axis
        elif key == Key.right:
            self.vel.linear.y -= self.step
        elif key == Key.left:
            self.vel.linear.y += self.step

        # down and up by z-axis
        elif has_char and key.char == "s":
            self.vel.linear.z -= self.step
        elif has_char and key.char == "w":
            self.vel.linear.z += self.step

        # yaw ?
        elif has_char and key.char == "a":
            self.vel.angular.z += self.step
        elif has_char and key.char == "d":
            self.vel.angular.z -= self.step

        else:
            self.vel.linear.x = 0.0
            self.vel.linear.y = 0.0
            self.vel.linear.z = 0.0
            self.vel.angular.x = 0.0
            self.vel.angular.y = 0.0
            self.vel.angular.z = 0.0

    # Create Keyboard Listener Thread
    # @See: https://pynput.readthedocs.io/en/latest/keyboard.html
    def run(self):
        # non-blocking mode for threads
        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()

        # blocking mode for threads
        # with keyboard.Listener(on_press=self.on_press) as listener:
        #     listener.join() # blocking mode for threads

def main(args=None):
    rclpy.init(args=args)

    try:
        node = TeleopControl()
        node.get_logger().info('Starting client, shut down with CTRL-C')
        rclpy.spin(node)

    except KeyboardInterrupt:
        if 'node' in locals():
            node.get_logger().info('Keyboard interrupt, shutting down.\n')

    except Exception as e:
        print(f'An error occurred: {e}')
        traceback.print_tb(e.__traceback__)

    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
