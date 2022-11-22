#!/usr/bin/env python3

from loopo_gripper.LoopO_driver import *
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from std_msgs.msg import Float32, Int32, Bool

gripper = loopo_driver()

node_update_timer_cb_group = ReentrantCallbackGroup()
loopo_update_timer_cb_group = MutuallyExclusiveCallbackGroup()

class LoopOUpdaterNode(Node):
    def __init__(self):
        super().__init__('loopo_updater')

        update_period = 0.01
        self.update_timer = self.create_timer(update_period,self.update_callback, callback_group = loopo_update_timer_cb_group)
    
    def update_callback(self):
        gripper.update()


class LoopOExtensionNode(Node):
    def __init__(self):
        super().__init__('loopo_extension_node')
        self.endtsop_publisher = self.create_publisher(Bool, 'loopo/extension/endstop', 10, callback_group = node_update_timer_cb_group)
        self.position_publisher = self.create_publisher(Int32, 'loopo/extension/position', 10, callback_group = node_update_timer_cb_group)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period,self.timer_callback, callback_group = loopo_update_timer_cb_group)
    
    def timer_callback(self):
        endstop = Bool()
        position = Int32()

        endstop.data = gripper.ex_endstop
        position.data = gripper.ex_position

        self.endtsop_publisher.publish(endstop)
        self.position_publisher.publish(position)
    
        
class LoopOTwistNode(Node):
    def __init__(self):
        super().__init__('loopo_twist_node')
        self.endtsop_publisher = self.create_publisher(Bool, 'loopo/twist/endstop', 10, callback_group = node_update_timer_cb_group)
        self.right_runout_publisher = self.create_publisher(Bool, 'loopo/twist/right/runout', 10, callback_group = node_update_timer_cb_group)
        self.left_runout_publisher = self.create_publisher(Bool, 'loopo/twist/left/runout', 10, callback_group = node_update_timer_cb_group)
        self.right_position_publisher = self.create_publisher(Int32, 'loopo/twist/right/position', 10, callback_group = node_update_timer_cb_group)
        self.left_position_publisher = self.create_publisher(Int32, 'loopo/twist/left/position', 10, callback_group = node_update_timer_cb_group)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period,self.timer_callback, callback_group = loopo_update_timer_cb_group)

    
    def timer_callback(self):
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
        self.runout_publisher = self.create_publisher(Bool, 'loopo/loop/runout', 10, callback_group = node_update_timer_cb_group)
        self.force_publisher = self.create_publisher(Float32, 'loopo/loop/force', 10, callback_group = node_update_timer_cb_group)
        self.position_publisher = self.create_publisher(Int32, 'loopo/loop/position', 10, callback_group = node_update_timer_cb_group)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period,self.timer_callback, callback_group = loopo_update_timer_cb_group)

    def timer_callback(self):

        runout = Bool()
        force = Float32()
        position = Int32()

        print(gripper.force)

        runout.data = gripper.lp_runout
        force.data = gripper.force
        position.data = gripper.lp_position

        self.runout_publisher.publish(runout)
        self.force_publisher.publish(force)
        self.position_publisher.publish(position)


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    #executor = SingleThreadedExecutor()

    loopo_updater_nodelet = LoopOUpdaterNode()
    loopo_extension_nodelet = LoopOExtensionNode()
    loopo_twist_nodelet = LoopOTwistNode()
    loopo_loop_nodelet = LoopOLoopNode()

    executor.add_node(loopo_updater_nodelet)
    executor.add_node(loopo_extension_nodelet)    
    executor.add_node(loopo_twist_nodelet)
    executor.add_node(loopo_loop_nodelet)

    rclpy.spin(executor)
    
    loopo_updater_nodelet.destroy_node()
    loopo_extension_nodelet.destroy_node()
    loopo_twist_nodelet.destroy_node()
    loopo_loop_nodelet.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()