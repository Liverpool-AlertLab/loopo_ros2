from loopo_control_terms import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool

from loopo_gripper.srv._send_loop_o_command import SendLoopOCommand
from loopo_gripper.action import Home, MoveTo

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

class ExtensionHomeServer(Node):
    def __init__(self):
        super.__init__('extension_home_server')
        self.position = 0
        self.endstop = 0
        self.home_as = ActionServer(self, Home, 'loopo/extension/home', self.home_cb)

    def home_cb(self, goal_handle):
        self.get_logger().info("homing motor...")

        start_time = self.get_clock().now()
        start_position = 0

        feedback_msg = Home.Feedback()
        feedback_msg.elapsed_time = 0
        feedback_msg.distace_moved = 0

        command_client = self.create_client(SendLoopOCommand, 'loopo/command')
        position_sub = self.create_subscription(Int32,'/loopo/extension/position')
        endstop_sub = self.

        homing_speed = goal_handle.speed

        result = Home.Result()
        
        command = SendLoopOCommand()
        command.Request.id = EX
        command_client.call(SendLoopOCommand()) (EX, TORQUE_ENABLE, 1)  
        gripper.send_command(EX, GOAL_SPEED, -homing_speed)  
        if gripper.ex_control_flag != NO_CONTROL:
            gripper.send_command(EX, CONTROL_APPROACH, NO_CONTROL)
            self.control_flag = NO_CONTROL
        while gripper.ex_endstop:
            pass
        gripper.send_command(EX, GOAL_SPEED, 0)
        gripper.send_command(EX, ZERO_ENCODER, 0)
        result.error_message = "no error"
        result._is_homed = True

    def position_sub_cb():
