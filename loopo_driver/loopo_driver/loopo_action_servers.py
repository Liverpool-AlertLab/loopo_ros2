#!/usr/bin/env python3

import rclpy

from loopo_driver.loopo_driver import LoopODriver

from rclpy.action import ActionServer
from rclpy.node import Node

from loopo_interfaces.action import Move, Homing, Grasp, Twist


class LoopOActionServers(Node):
    def __init__(self, loopo_driver=LoopODriver):
        super().__init__("loopo_action_servers")

        self.gripper = loopo_driver

        self.loop_move_action_server = ActionServer(
            self, Move, "move_loop", self.loop_move_callback
        )

        self.loop_home_action_server = ActionServer(
            self, Homing, "homing_loop", self.loop_home_callback
        )

        self.loop_grasp_action_server = ActionServer(
            self, Grasp, "grasp_loop", self.loop_grasp_callback
        )

        self.get_logger().info("Loop-O action servers started and ready.")

    def loop_move_callback(self, goal_handle):
        self.gripper.enable(self.gripper.LOOP)
        self.gripper.switch_to_position_control(self.gripper.LOOP)
        self.goal_size = float(goal_handle.request.target)
        feedback = Move.Feedback()
        result = Move.Result()
        self.gripper.set_loop_size(self.goal_size)
        self.gripper.update()
        feedback.current_position = float(self.gripper.loop_size)
        goal_handle.publish_feedback(feedback)

        while self.gripper.loop_status == 3:
            self.gripper.update()
            feedback.current_position = float(self.gripper.loop_size)
            goal_handle.publish_feedback(feedback)

        if self.gripper.loop_status == 1:
            result.success = True
            result.error = ""
            goal_handle.succeed()
        elif self.gripper.loop_status == 2:
            result.success = False
            result.error = "Loop is homing"
            goal_handle.abort()
        elif self.gripper.loop_status == 4:
            result.success = False
            result.error = "Loop hit the endstop"
            goal_handle.abort()
        elif self.gripper.loop_status == 5:
            result.success = False
            result.error = "Loop is stuck"
            self.get_logger().info("Error: %s" % (result.error))
            goal_handle.abort()

        return result

    def loop_home_callback(self, goal_handle):
        self.gripper.enable(self.gripper.LOOP)
        self.homing_speed = float(goal_handle.request.speed)
        feedback = Homing.Feedback()
        result = Homing.Result()
        self.gripper.home_loop(self.homing_speed)
        feedback.current_position = float(self.gripper.loop_size)
        goal_handle.publish_feedback(feedback)

        while self.gripper.loop_status == 2:
            self.gripper.update()
            feedback.current_position = float(self.gripper.loop_size)
            goal_handle.publish_feedback(feedback)

        if self.gripper.loop_status == 1:
            result.success = True
            result.error = ""
            goal_handle.succeed()
        elif self.gripper.loop_status == 4:
            result.success = True
            result.error = "Loop hit the endstop"
            goal_handle.succeed()
        elif self.gripper.loop_status == 5:
            result.success = False
            result.error = "Loop is stuck"
            goal_handle.abort()

        return result

    def loop_grasp_callback(self, goal_handle):
        self.gripper.enable(self.gripper.LOOP)
        self.gripper.switch_to_position_control(self.gripper.LOOP)
        self.goal_width = float(goal_handle.request.width)
        self.goal_force = float(goal_handle.request.force)
        feedback = Grasp.Feedback()
        result = Grasp.Result()
        self.gripper.send_command(3, 3, self.goal_width)
        self.gripper.update()
        feedback.current_width = float(self.gripper.loop_size)
        feedback.current_force = float(self.gripper.force)
        goal_handle.publish_feedback(feedback)

        while self.gripper.loop_status == 3:
            self.gripper.update()
            feedback.current_width = float(self.gripper.loop_size)
            feedback.current_force = float(self.gripper.force)
            goal_handle.publish_feedback(feedback)

        if self.gripper.loop_status == 2:
            result.success = False
            result.error = "Loop is homing"
            goal_handle.abort()
        elif self.gripper.loop_status == 4:
            result.success = False
            result.error = "Loop hit the endstop"
            goal_handle.abort()

        self.gripper.set_force(self.goal_force)
        self.gripper.switch_to_force_control()

        while self.gripper.force < self.goal_force:
            self.gripper.update()

            feedback.current_width = float(self.gripper.loop_size)
            feedback.current_force = float(self.gripper.force)
            goal_handle.publish_feedback(feedback)

        result.success = True
        result.error = ""
        self.get_logger().info("Error: %s" % (result.error))
        goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)

    loopo = LoopODriver(com_port="/dev/ttyACM0")

    extension_action_server = LoopOActionServers(loopo_driver=loopo)

    rclpy.spin(extension_action_server)


if __name__ == "__main__":
    main()
