#!/usr/bin/env python3

from loopo_driver.loopo_driver import LoopODriver

from loopo_interfaces.srv import SendCommand

import rclpy
from rclpy.node import Node


class LoopODriverNode(Node):
    def __init__(self):
        super().__init__("loopo_driver_node")

        self.gripper = LoopODriver()
        self.command_service = self.create_service(
            SendCommand, "loopo_command", self.send_command
        )

    def gripper_update(self):
        self.gripper.update()
        return 1

    def send_command(self, request, response):
        self.gripper.send_command(request.id, request.command, request.value)

        response.lp_size = self.gripper.lp_size
        response.lp_force = self.gripper.force
        response.lp_status = self.gripper.lp_status
        response.lp_control = self.gripper.lp_control

        return response


def main(args=None):
    rclpy.init(args=args)

    loopo_driver_node = LoopODriverNode()

    rclpy.spin(loopo_driver_node)

    loopo_driver_node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
