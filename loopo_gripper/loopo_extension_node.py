
from loopo_gripper.action import Home, MoveTo
from loopo_gripper.srv._enable_disable_actuator import EnableDisableActuator

from std_msgs.msg import Float32, Int32, Bool

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

class LoopOExtensionNode(Node):
    def __init__(self):
        super().__init__('loopo_extension_node')

        self.endstop_subscriber = self.create_subscription(Bool, 'loopo/extension/endstop', 10, callback_group = ros_operation_cb_group)
        self.position_publisher = self.create_publisher(Int32, 'loopo/extension/position', 10, callback_group = ros_operation_cb_group)

        self.enable_disable_torque_srv = self.create_service(EnableDisableActuator, 'loopo/extension/enable_diasble_torque', self.enable_disable_torque_cb, callback_group = ros_operation_cb_group)

        self.home_as = ActionServer(self, Home, 'loopo/extension/home', self.home_cb, callback_group = mcu_operation_cb_group)
        self.move_to_as = ActionServer(self, MoveTo, 'loopo/extension/move_to', self.move_to_cb, callback_group = mcu_operation_cb_group)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period,self.timer_cb, callback_group = ros_operation_cb_group)

        self.torque_flag = 0
        self.control_flag = NO_CONTROL

        self.get_logger().info("LoopO Extension node initialised")
    
    def timer_cb(self):
        endstop = Bool()
        position = Int32()

        endstop.data = gripper.ex_endstop
        position.data = gripper.ex_position

        self.endtsop_publisher.publish(endstop)
        self.position_publisher.publish(position)

    def enable_disable_torque_cb(self, request, response):
        if request.enable:
            self.get_logger().info("disabling motor")
            if self.torque_flag:
                pass
            else:
                gripper.send_command(EX, TORQUE_ENABLE, 1)
            response.status = True
        if request.disable:
            self.get_logger().info("enabling motor")
            if not self.torque_flag:
                pass
            else:
                gripper.send_command(EX, TORQUE_ENABLE, 0)
            response.status = False
        return response

    def home_cb(self, goal_handle):
        self.get_logger().info("homing motor")
        homing_speed = goal_handle.speed
        result = Home.Result()
        feedback = Home.Feedback()
        if not self.torque_flag:
            gripper.send_command(EX, TORQUE_ENABLE, 1)  
        gripper.send_command(EX, GOAL_SPEED, -homing_speed)  
        if self.control_flag != NO_CONTROL:
            gripper.send_command(EX, CONTROL_APPROACH, NO_CONTROL)
            self.control_flag = NO_CONTROL
        while gripper.ex_endstop:
            pass
        gripper.send_command(EX, GOAL_SPEED, 0)
        gripper.send_command(EX, ZERO_ENCODER, 0)
        result.error_message = "no error"
        result._is_homed = True

    def move_to_cb(self, goal_handle):
        self.get_logger().info("moving motor")
        target = goal_handle.target
        tolerance = goal_handle.tolerance
        result = MoveTo.Result()
        gripper.send_command(EX, GOAL_POSITION, target)
        if not self.control_flag != POSITION_CONTROL:
            gripper.send_command(EX, CONTROL_APPROACH, POSITION_CONTROL)
        if not self.torque_flag:
            gripper.send_command(EX, TORQUE_ENABLE, 1)
        while (gripper.ex_position <= target - tolerance) or (gripper.ex_position >= target - tolerance):
            pass
        result.error_message = 'no error'
        result.status = gripper.ex_position
        result.target_reached = True   