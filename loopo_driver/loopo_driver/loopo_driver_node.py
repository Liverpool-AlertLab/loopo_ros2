#!/usr/bin/env python3

from loopo_driver.loopo_driver import LoopODriver

from loopo_messages.msg import ExtensionActuatorUpdate, LoopActuatorUpdate, TwistActuatorUpdate, LoopOCommand
from loopo_messages.srv import SendLoopOCommand

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

#gripper = LoopODriver()

parallel_group = ReentrantCallbackGroup()
#mcu_operation_cb_group = MutuallyExclusiveCallbackGroup()

class LoopODriverNode(Node):
    def __init__(self):
        super().__init__('loopo_driver_node')

        self.gripper = LoopODriver()
        
        self.extension_actuator_publisher = self.create_publisher(ExtensionActuatorUpdate, 'loopo/extension_actuator/status', 10)
        self.twist_actuator_publisher = self.create_publisher(TwistActuatorUpdate, 'loopo/twist_actuator/status', 10)
        self.loop_actuator_publisher = self.create_publisher(LoopActuatorUpdate, 'loopo/loop_actuator/status', 10)

        self.update_timer = self.create_timer(0.01, self.gripper_update, parallel_group)
        self.publisher_timer = self.create_timer(0.1, self.publisher_callback, parallel_group)
        
        self.rate = self.create_rate(0.005)

        self.command_service = self.create_service(SendLoopOCommand, 'loopo_command', self.send_command, callback_group=parallel_group)

    def gripper_update(self):
        self.gripper.update()
        return 1

    def send_command(self, request, response):
        self.gripper.send_command(request.id, request.command, request.value)
        while (self.gripper.ack == 0):
            self.gripper_update()
            self.get_logger().info('ACK: %d' % (self.gripper.ack))
            self.rate.sleep()
        response.sent = True
        return response

    def publisher_callback(self):
        extension_msg = ExtensionActuatorUpdate()
        loop_msg = LoopActuatorUpdate()
        twist_msg = TwistActuatorUpdate()

        extension_msg.position = self.gripper.ex_position
        extension_msg.status = self.gripper.ex_status
        extension_msg.control = self.gripper.ex_control

        loop_msg.size = self.gripper.lp_position
        loop_msg.force = self.gripper.force
        loop_msg.status = self.gripper.lp_status
        loop_msg.control = self.gripper.lp_control

        twist_msg.size = self.gripper.tw_position
        twist_msg.offset = self.gripper.tw_offset
        twist_msg.status = self.gripper.tw_statuts
        twist_msg.control = self.gripper.tw_control

        self.extension_actuator_publisher.publish(extension_msg)
        self.twist_actuator_publisher.publish(twist_msg)
        self.loop_actuator_publisher.publish(loop_msg)

        return 1

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    loopo_driver_node = LoopODriverNode()

    #rclpy.spin(loopo_driver_node)

    executor.add_node(loopo_driver_node)

    executor.spin()
    
    loopo_driver_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()