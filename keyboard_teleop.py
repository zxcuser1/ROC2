#!/usr/bin/env python3
import sys
import random
# pynput throws an error if we import it before $DISPLAY is set on LINUX
from pynput.keyboard import KeyCode, Listener, Key
from pynput import keyboard
if sys.platform not in ('darwin', 'win32'):
    import os

    os.environ.setdefault('DISPLAY', ':0')

import rclpy
from rclpy.node import Node
# from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped
from builtin_interfaces.msg import Time

class KeystrokeListen:
    def __init__(self, name=None):
        self.node = rclpy.create_node('keyboard_node')
        self.pub = self.node.create_publisher(TwistStamped, "/controller/reference", 10)
        self.key_state_vel_x = 0.0
        self.key_state_vel_y = 0.0
        self.key_state_ang = 0.0
        
        self.backwards = False

        if self.backwards == True:
            self.MYROBOT_MAX_LIN_VEL_X = -1.0
            self.MYROBOT_MAX_LIN_VEL_Y = -1.0
            self.MYROBOT_MAX_ANG_VEL = -1.0
        else:
            self.MYROBOT_MAX_LIN_VEL_X = 1.0
            self.MYROBOT_MAX_LIN_VEL_Y = 1.0
            self.MYROBOT_MAX_ANG_VEL = 1.0

    def spin(self):
        self.node.get_logger().info("Listen keys...")
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            while rclpy.ok() and listener.running:
                rclpy.spin_once(self.node, timeout_sec=0.8)
                listener.join()

    def update_key_state(self): 
        #обновление состояния клавиши
        twist = TwistStamped()
        twist.header.stamp = self.node.get_clock().now().to_msg()
        twist.twist.linear.x = self.key_state_vel_x; twist.twist.linear.y = self.key_state_vel_y; twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0; twist.twist.angular.y = 0.0; twist.twist.angular.z = self.key_state_ang
        self.pub.publish(twist)

    def on_release(self, key): 
        #обработка отпускания клавиши
    
        if key == KeyCode.from_char('w'):
            self.key_state_vel_x = 0.0

        elif key == KeyCode.from_char('s'):
            self.key_state_vel_x = 0.0

        elif key == KeyCode.from_char('q'):
            self.key_state_vel_y = 0.0

        elif key == KeyCode.from_char('e'):
            self.key_state_vel_y = 0.0

        elif key == KeyCode.from_char('a'):
            self.key_state_ang = 0.0

        elif key == KeyCode.from_char('d'):
            self.key_state_ang = 0.0

        self.update_key_state()
        return True

    def on_press(self, key): 
        #обработка нажатия клавиши
        step = 0.1 + random.random()

        if key == Key.space:
            self.key_state_vel_x = max(self.key_state_vel_x - step, 0.0)
            self.key_state_vel_y = max(self.key_state_vel_y - step, 0.0)
            self.key_state_ang = max(self.key_state_ang - step, 0.0)

        elif key == KeyCode.from_char('w'):
            self.key_state_vel_x = min(self.key_state_vel_x + step,
            self.MYROBOT_MAX_LIN_VEL_X)

        elif key == KeyCode.from_char('s'):
            self.key_state_vel_x = max(self.key_state_vel_x - step, 
            -self.MYROBOT_MAX_LIN_VEL_X)

        elif key == KeyCode.from_char('q'):
            self.key_state_vel_y = min(self.key_state_vel_y + step,
            self.MYROBOT_MAX_LIN_VEL_Y)

        elif key == KeyCode.from_char('e'):
            self.key_state_vel_y = max(self.key_state_vel_y - step, 
            -self.MYROBOT_MAX_LIN_VEL_Y)

        elif key == KeyCode.from_char('a'):
            self.key_state_ang = self.MYROBOT_MAX_ANG_VEL

        elif key == KeyCode.from_char('d'):
            self.key_state_ang = -self.MYROBOT_MAX_ANG_VEL
        
        self.update_key_state()
        return True

def main(args=None):
    rclpy.init(args=args)
    KeystrokeListen().spin()

if __name__ == '__main__':
    main()

