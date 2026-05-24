#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Control Your racecar!
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

space key, k : force stop
w/x: shift the middle pos of throttle by +/- 5 pwm
a/d: shift the middle pos of steering by +/- 2 pwm
CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, -1),
    'm': (-1, 1),
    'I': (1, 0),
    'O': (1, -1),
    'J': (0, 1),
    'L': (0, -1),
    'U': (1, 1),
    '<': (-1, 0),
    '>': (-1, -1),
    'M': (-1, 1),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

class RacecarTeleop(Node):
    def __init__(self):
        super().__init__('racecar_teleop')
        self.pub = self.create_publisher(Twist, '/teleop_cmd_vel', 5)
        # self.pub = self.create_publisher(Twist, '/cmd_vel', 5)
        # self.pub = self.create_publisher(Twist, '/car_cmd_vel', 5)

        self.settings = termios.tcgetattr(sys.stdin)
        # self.speed_start_value = 50  # can roll
        self.speed_start_value = 23  # can roll
        self.turn_start_value = 60  # max value
        self.speed_mid = 1500
        self.turn_mid = 90
        self.speed_bias = 0
        self.turn_bias = 0
        self.speed_add_once = 1
        self.turn_add_once = 2
        self.control_speed = self.speed_mid
        self.control_turn = self.turn_mid
        self.speed_dir = 0
        self.last_speed_dir = 0
        self.last_turn_dir = 0
        self.last_control_speed = self.control_speed
        self.last_control_turn = self.control_turn
        self.run = 0

    def run_teleop(self):
        try:
            while True:
                key = getKey(self.settings)
                twist = Twist()
                if key in moveBindings.keys():
                    self.run = 1
                    self.speed_dir = moveBindings[key][0]
                    if self.speed_dir != 0 and self.speed_dir + self.last_speed_dir == 0:  # Reverse
                        self.control_speed = self.speed_mid
                        self.last_speed_dir = self.speed_dir
                        self.control_turn = self.turn_mid
                    else:
                        if self.speed_dir > 0:
                            self.control_speed = self.speed_dir * (self.speed_start_value + self.speed_bias) + self.speed_mid
                        elif self.speed_dir < 0:
                            self.control_speed = self.speed_dir * (self.speed_start_value + self.speed_bias) + self.speed_mid - 140
                        else:
                            self.control_speed = 1500
                        self.control_turn = moveBindings[key][1] * (self.turn_start_value + self.turn_bias) + self.turn_mid
                        self.last_speed_dir = self.speed_dir
                    self.last_control_speed = self.control_speed
                    self.last_control_turn = self.control_turn
                elif key == ' ' or key == 'k':
                    self.speed_dir = -self.speed_dir  # for ESC Forward/Reverse with brake mode
                    self.control_speed = self.last_control_speed * self.speed_dir
                    self.control_turn = self.turn_mid
                    self.run = 0
                elif key == 'w':
                    self.speed_bias += self.speed_add_once
                    if self.speed_bias >= 450:
                        self.speed_bias = 450
                    else:
                        self.last_control_speed = self.last_control_speed + self.speed_add_once
                    self.run = 0
                elif key == 's':
                    self.speed_bias -= self.speed_add_once
                    self.last_control_speed = self.last_control_speed - self.speed_add_once
                    if self.speed_bias <= 0:
                        self.speed_bias = 0
                    self.run = 0
                elif key == 'a':
                    self.turn_bias += self.turn_add_once
                    self.last_control_turn = self.last_control_turn + self.turn_add_once
                    if self.turn_bias >= 80:
                        self.turn_bias = 80
                    self.run = 0
                elif key == 'd':
                    self.turn_bias -= self.turn_add_once
                    self.last_control_turn = self.last_control_turn - self.turn_add_once
                    if self.turn_bias <= 0:
                        self.turn_bias = 0
                    self.run = 0
                else:
                    self.run = 0
                print(vels(self.control_speed, self.control_turn))
                if self.run == 1:
                    twist.linear.x = float(self.control_speed)
                    twist.angular.z = float(self.control_turn)
                    self.pub.publish(twist)
                else:
                    twist.linear.x = float(self.speed_mid)
                    twist.angular.z = float(self.turn_mid)
                    self.pub.publish(twist)
                if key == '\x03':  # for ctrl + c exit
                    break
        except Exception as e:
            print(e)
        finally:
            twist = Twist()
            twist.linear.x = float(self.speed_mid)
            twist.angular.z = float(self.turn_mid)
            self.pub.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = RacecarTeleop()
    node.run_teleop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
