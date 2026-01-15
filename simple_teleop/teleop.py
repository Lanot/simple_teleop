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
        atexit.register(self.emergency_stop)

        super().__init__('teleop_control')

        self.declare_parameter('topic', '/cmd_vel')
        self.declare_parameter("step", 0.1)
        self.declare_parameter("publish_rate", 10)


        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.step = self.get_parameter("step").value
        self.publish_rate = 1 / self.get_parameter("publish_rate").value

        self.vel = Twist()
        self.vel_pub = self.create_publisher(Twist, self.topic, 10)
        self.create_timer(self.publish_rate, self.publish_teleop)

        self.print_info()

        self.run()

    def emergency_stop(self):
        self.vel_pub.publish(Twist())

    # 20Hz - works good
    def publish_teleop(self):
        self.vel_pub.publish(self.current_vel_cm)

    def print_info(self):
        self.get_logger().info("\n\tUse Arrows to increment linear velocities by x and y axes, +/- - to up and down\n\
            a/d,\n\
            w/s,\n\
            z/x,\n\
            space: zero velocity command,\n\
            ~esc: QUIT")

    def on_press(self, key):
        has_char = hasattr(key, 'char')

        if key == Key.esc:
            print("~ESC~ key clicked")
            raise KeyboardInterrupt()

        # move backward / forward by x-axis
        elif key == Key.up:
            print("Up key clicked")
            self.vel.linear.x += self.step
        elif key == Key.down:
            print("Down key clicked")
            self.vel.linear.x -= self.step

        # move backward / forward by y-axis
        elif key == Key.right:
            print("Right key clicked")
            self.vel.linear.y -= self.step
        elif key == Key.left:
            print("Left key clicked")
            self.vel.linear.y += self.step

        # down and up by z-axis
        elif has_char is not None and key.char == "-":
            print("- key clicked")
            self.vel.linear.z -= self.step
        elif has_char is not None and key.char == "=":
            print("+ key clicked")
            self.vel.linear.z += self.step

        # yaw ?
        elif has_char is not None and key.char == "a":
            print("a key clicked")
            self.vel.angular.z -= self.step
        elif has_char is not None and key.char == "d":
            print("d key clicked")
            self.vel.angular.z += self.step

        # pitch ?
        elif has_char is not None and key.char == "w":
            print("w key clicked")
            self.vel.angular.y -= self.step
        elif has_char is not None and key.char == "s":
            print("s key clicked")
            self.vel.angular.y += self.step

        # roll ?
        elif has_char is not None and key.char == "z":
            print("z key clicked")
            self.vel.angular.x -= self.step
        elif has_char is not None and key.char == "x":
            print("x key clicked")
            self.vel.angular.x += self.step

        else:
            self.vel.linear.x = 0
            self.vel.linear.y = 0
            self.vel.linear.z = 0
            self.vel.angular.x = 0
            self.vel.angular.y = 0
            self.vel.angular.z = 0

    # Create Listener Thread
    def run(self):
        with keyboard.Listener(on_press=self.on_press) as listener:
            listener.join()

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
