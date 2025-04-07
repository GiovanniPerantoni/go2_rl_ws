#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_go.msg import WirelessController
from rclpy.duration import Duration
from std_msgs.msg import Float32MultiArray

import pygame


class WiredControlPublisher(Node):
    def __init__(self, joycon):
        super().__init__('wired_control_publisher')
        self.joycon = joycon
        self.publisher_ = self.create_publisher(WirelessController, 'wiredcontroller', 10)
        self.timer = self.create_timer(0.01, self.wired_controller_callback)

    def wired_controller_callback(self):
        pygame.event.pump()

        axes = [self.joycon.get_axis(i) for i in range(self.joycon.get_numaxes())]
        hats = [self.joycon.get_hat(i) for i in range(self.joycon.get_numhats())]
        buttons = [self.joycon.get_button(i) for i in range(self.joycon.get_numbuttons())]

        msg = WirelessController()
        msg.lx = 0.0
        msg.ly = 0.0
        msg.rx = 0.0
        msg.ry = 0.0
        msg.keys = 0

        # Joysticks data
        trigger_treshold = 0.06
        if axes[0]>trigger_treshold or axes[0]<-trigger_treshold: 
            msg.lx = axes[0]
        if axes[1]>trigger_treshold or axes[1]<-trigger_treshold: 
            msg.ly = -axes[1]
        if axes[2]>trigger_treshold or axes[2]<-trigger_treshold: 
            msg.rx = axes[2]
        if axes[3]>trigger_treshold or axes[3]<-trigger_treshold: 
            msg.ry = -axes[3]

        # Pressed button data
        # TODO: fix spagetthi code
        if buttons[7] == 0 or buttons[2] == 1:
            msg.keys = 512
        elif buttons[1] == 1:
            msg.keys = 256
        elif hats[0][1] == 1:
            msg.keys = 4096
        elif hats[0][1] == -1:
            msg.keys = 16384
        elif buttons[9] == 1:
            msg.keys = 4
        elif buttons[8] == 1:
            msg.keys = 8

        self.publisher_.publish(msg)


def main(args=None):
    # Initialize pygame
    pygame.init()
    pygame.joystick.init()
    # Check for connected joysticks
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No joysticks detected")
        exit()
    # Initialize the first joystick (Joy-Con)
    joycon = pygame.joystick.Joystick(0)
    joycon.init()
    # Create node and spin it
    rclpy.init()
    node = WiredControlPublisher(joycon)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutwond()
    # Quit pygame
    joycon.quit()
    pygame.quit()


if __name__ == "__main__":
    main()

