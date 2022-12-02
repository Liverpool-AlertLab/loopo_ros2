#!/usr/bin/env python3

from loopo_gripper.LoopO_driver import *

from loopo_gripper.srv._enable_disable_actuator import EnableDisableActuator, EnableDisableActuator_Request, EnableDisableActuator_Response

from loopo_gripper.action import Home, MoveTo

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from std_msgs.msg import Float32, Int32, Bool

gripper = loopo_driver()

ros_operation_cb_group = ReentrantCallbackGroup()
mcu_operation_cb_group = MutuallyExclusiveCallbackGroup()

class LoopOUpdaterNode(Node):
    def __init__(self):
        super().__init__('loopo_updater')

        update_period = 0.01
        self.update_timer = self.create_timer(update_period,self.update_cb, callback_group = mcu_operation_cb_group)

        self.get_logger().info("LoopO Updater node initialised")

    def update_cb(self):
        gripper.update()
        #self.get_logger().info('ex_es: %d - tw_es: %d - tr_ro: %d - tl_ro: %d - lp_fo: %f - ex_po: %d - tr_po: %d - tl_po: %d - lp_po: %d' % (gripper.ex_endstop, gripper.tw_endstop, gripper.tr_runout, gripper.tl_runout, gripper.force, gripper.ex_position, gripper.tr_position, gripper.tl_position, gripper.lp_position))


class LoopOExtensionNode(Node):
    def __init__(self):
        super().__init__('loopo_extension_state')

        self.endtsop_publisher = self.create_publisher(Bool, 'loopo/extension/endstop', 10, callback_group = ros_operation_cb_group)
        self.position_publisher = self.create_publisher(Int32, 'loopo/extension/position', 10, callback_group = ros_operation_cb_group)

        self.enable_disable_torque_srv = self.create_service(EnableDisableActuator, 'loopo/extension/enable_diasble_torque', self.enable_disable_torque_cb, callback_group = ros_operation_cb_group)

        self.home_as = ActionServer(self, Home, 'loopo/extension/home', self.home_cb, callback_group = mcu_operation_cb_group)
        self.move_to_as = ActionServer(self, MoveTo, 'loopo/extension/move_to', self.move_to_cb, callback_group = mcu_operation_cb_group)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period,self.timer_cb, callback_group = ros_operation_cb_group)

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
            if gripper.ex_torque_flag:
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

class ExtensionHomeServer(Node):
    def __init__(self):
        super.__init__('extension_home_server')
        self.home_as = ActionServer(self, Home, 'loopo/extension/home', self.home_cb, callback_group = mcu_operation_cb_group)

    def home_cb(self, goal_handle):
        self.get_logger().info("homing motor")
        homing_speed = goal_handle.speed
        result = Home.Result()
        feedback = Home.Feedback()
        if not gripper.ex_torque_flag:
            gripper.send_command(EX, TORQUE_ENABLE, 1)  
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

class ExtensionHomeServer(Node):
    def __init__(self):
        super.__init__('extension_move_to_server')
        self.home_as = ActionServer(self, Home, 'loopo/extension/move_to', self.home_cb, callback_group = mcu_operation_cb_group)

    def move_to_cb(self, goal_handle):
        self.get_logger().info("moving motor")
        target = goal_handle.target
        tolerance = goal_handle.tolerance
        result = MoveTo.Result()
        gripper.send_command(EX, GOAL_POSITION, target)
        if not gripper.ex_control_flag != POSITION_CONTROL:
            gripper.send_command(EX, CONTROL_APPROACH, POSITION_CONTROL)
        if not gripper.ex_torque_flag:
            gripper.send_command(EX, TORQUE_ENABLE, 1)
        while (gripper.ex_position <= target - tolerance) or (gripper.ex_position >= target - tolerance):
            pass
        result.error_message = 'no error'
        result.status = gripper.ex_position
        result.target_reached = True
      
        
class LoopOTwistNode(Node):
    def __init__(self):
        super().__init__('loopo_twist_node')
        self.endtsop_publisher = self.create_publisher(Bool, 'loopo/twist/endstop', 10, callback_group = ros_operation_cb_group)
        self.right_runout_publisher = self.create_publisher(Bool, 'loopo/twist/right/runout', 10, callback_group = ros_operation_cb_group)
        self.left_runout_publisher = self.create_publisher(Bool, 'loopo/twist/left/runout', 10, callback_group = ros_operation_cb_group)
        self.right_position_publisher = self.create_publisher(Int32, 'loopo/twist/right/position', 10, callback_group = ros_operation_cb_group)
        self.left_position_publisher = self.create_publisher(Int32, 'loopo/twist/left/position', 10, callback_group = ros_operation_cb_group)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period,self.timer_cb, callback_group = ros_operation_cb_group)

        self.get_logger().info("LoopO Twist node initialised")

    
    def timer_cb(self):
        endstop = Bool()
        right_runout = Bool()
        left_runout = Bool()
        right_position = Int32()
        left_position = Int32()

        endstop.data = gripper.tw_endstop
        right_runout.data = gripper.tr_runout
        left_runout.data = gripper.tl_runout
        right_position.data = gripper.tr_position
        left_position.data = gripper.tl_position

        self.endtsop_publisher.publish(endstop)
        self.right_runout_publisher.publish(right_runout) 
        self.left_runout_publisher.publish(left_runout) 
        self.right_position_publisher.publish(left_position)
        self.left_position_publisher.publish(left_position)


class LoopOLoopNode(Node):
    def __init__(self):
        super().__init__('loopo_loop_node')
        self.runout_publisher = self.create_publisher(Bool, 'loopo/loop/runout', 10, callback_group = ros_operation_cb_group)
        self.force_publisher = self.create_publisher(Float32, 'loopo/loop/force', 10, callback_group = ros_operation_cb_group)
        self.position_publisher = self.create_publisher(Int32, 'loopo/loop/position', 10, callback_group = ros_operation_cb_group)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period,self.timer_cb, callback_group = ros_operation_cb_group)

        self.get_logger().info("LoopO Loop node initialised")

    def timer_cb(self):

        runout = Bool()
        force = Float32()
        position = Int32()

        runout.data = gripper.lp_runout
        force.data = gripper.force
        position.data = gripper.lp_position

        self.runout_publisher.publish(runout)
        self.force_publisher.publish(force)
        self.position_publisher.publish(position)


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    loopo_updater_nodelet = LoopOUpdaterNode()
    loopo_extension_nodelet = LoopOExtensionNode()
    loopo_twist_nodelet = LoopOTwistNode()
    loopo_loop_nodelet = LoopOLoopNode()

    executor.add_node(loopo_updater_nodelet)
    executor.add_node(loopo_extension_nodelet)    
    executor.add_node(loopo_twist_nodelet)
    executor.add_node(loopo_loop_nodelet)

    executor.spin()
    
    loopo_updater_nodelet.destroy_node()
    loopo_extension_nodelet.destroy_node()
    loopo_twist_nodelet.destroy_node()
    loopo_loop_nodelet.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()