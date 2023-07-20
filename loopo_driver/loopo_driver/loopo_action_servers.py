#!/usr/bin/env python3

import rclpy

from loopo_driver.loopo_driver import LoopODriver

from rclpy.action import ActionServer
from rclpy.node import Node

from loopo_interfaces.action import Move, Homing, Grasp, Twist


class LoopOActionServers(Node):
    def __init__(self, loopo_driver=LoopODriver):
        super().__init__("extension_action_servers")

        self.gripper = loopo_driver

        self.extension_move_action_server = ActionServer(
            self, Move, "move_extension", self.extension_move_callback
        )

        self.extension_home_action_server = ActionServer(
            self, Homing, "homing_extension", self.extension_home_callback
        )

        self.loop_move_action_server = ActionServer(
            self, Move, "move_loop", self.loop_move_callback
        )

        self.loop_home_action_server = ActionServer(
            self, Homing, "homing_loop", self.loop_home_callback
        )

        self.loop_grasp_action_server = ActionServer(
            self, Grasp, "grasp_loop", self.loop_grasp_callback
        )

        self.twist_move_action_server = ActionServer(
            self, Move, "move_twist", self.twist_move_callback
        )

        self.twist_home_action_server = ActionServer(
            self, Homing, "homing_twist", self.twist_home_callback
        )

        self.twist_grasp_action_server = ActionServer(
            self, Grasp, "grasp_twist", self.twist_grasp_callback
        )

        self.twist_twist_action_server = ActionServer(
            self, Twist, "twist_twist", self.twist_twist_callback
        )

        self.get_logger().info("Loop-O action servers started and ready.")

    def extension_move_callback(self, goal_handle):
        self.gripper.enable(self.gripper.EXTENSION)
        self.gripper.switch_to_position_control(self.gripper.EXTENSION)

        self.goal = float(goal_handle.request.target)
        feedback = Move.Feedback()
        result = Move.Result()
        self.gripper.set_extension_position(self.goal)
        self.gripper.update()

        feedback.current_position = float(self.gripper.extension_position)
        goal_handle.publish_feedback(feedback)

        while self.gripper.extension_status == 3:
            self.gripper.update()

            feedback.current_position = float(self.gripper.extension_position)
            goal_handle.publish_feedback(feedback)

        if self.gripper.extension_status == 1:
            result.success = True
            result.error = ""
            goal_handle.succeed()
        elif self.gripper.extension_status == 2:
            result.success = False
            result.error = "Extension is homing"
            goal_handle.abort()
        elif self.gripper.extension_status == 4:
            result.success = False
            result.error = "Extension hit the endstop"
            goal_handle.abort()
        elif self.gripper.extension_status == 5:
            result.success = False
            result.error = "Extension is stuck"
            goal_handle.abort()

        return result

    def extension_home_callback(self, goal_handle):
        self.gripper.enable(self.gripper.EXTENSION)
        self.homing_speed = float(goal_handle.request.speed)
        feedback = Homing.Feedback()
        result = Homing.Result()
        self.gripper.home_extension(self.homing_speed)
        self.gripper.update()
        feedback.current_position = float(self.gripper.extension_position)
        goal_handle.publish_feedback(feedback)

        while self.gripper.extension_status == 2:
            self.gripper.update()
            feedback.current_position = float(self.gripper.extension_position)
            goal_handle.publish_feedback(feedback)

        if self.gripper.extension_status == 1:
            result.success = True
            result.error = ""
            goal_handle.succeed()
        elif self.gripper.extension_status == 4:
            result.success = True
            result.error = "Extension hit the endstop"
            goal_handle.succeed()
        elif self.gripper.extension_status == 5:
            result.success = False
            result.error = "Extension is stuck"
            goal_handle.abort()

        return result

    def twist_move_callback(self, goal_handle):
        self.gripper.enable(self.gripper.TWIST)
        self.gripper.switch_to_position_control(self.gripper.TWSIT)
        self.goal_size = float(goal_handle.request.target)
        feedback = Move.Feedback()
        result = Move.Result()
        self.gripper.set_twist_size(self.goal_size)
        self.gripper.update()
        feedback.current_position = float(self.gripper.twist_size)
        goal_handle.publish_feedback(feedback)

        while self.gripper.twist_status == 3:
            self.gripper.update()
            feedback.current_position = float(self.gripper.twist_size)
            goal_handle.publish_feedback(feedback)

        if self.gripper.twist_status == 1:
            result.success = True
            result.error = ""
            goal_handle.succeed()
        elif self.gripper.twist_status == 2:
            result.success = False
            result.error = "Twist is homing"
            goal_handle.abort()
        elif self.gripper.twist_status == 4:
            result.success = False
            result.error = "Twist hit the endstop"
            goal_handle.abort()
        elif self.gripper.twist_status == 5:
            result.success = False
            result.error = "Twist is stuck"
            goal_handle.abort()

        return result

    def twist_home_callback(self, goal_handle):
        self.gripper.enable(self.gripper.TWIST)
        self.homing_speed = float(goal_handle.request.speed)
        feedback = Homing.Feedback()
        result = Homing.Result()
        self.gripper.home_twist(self.homing_speed)
        feedback.current_position = float(self.gripper.twist_size)
        goal_handle.publish_feedback(feedback)
        self.gripper.send_command(0, 0, 0)

        while self.gripper.twist_status == 2:
            self.gripper.update()
            feedback.current_position = float(self.gripper.twist_size)
            goal_handle.publish_feedback(feedback)

        if self.gripper.twist_status == 1:
            result.success = True
            result.error = ""
            goal_handle.succeed()
        elif self.gripper.twist_status == 4:
            result.success = True
            result.error = "Twist hit the endstop"
            goal_handle.succeed()
        elif self.gripper.twist_status == 5:
            result.success = False
            result.error = "Twist is stuck"
            goal_handle.abort()

        return result

    def twist_grasp_callback(self, goal_handle):
        self.gripper.enable(self.gripper.TWIST)
        self.gripper.switch_to_position_control(self.gripper.TWIST)
        self.goal_width = float(goal_handle.request.width)
        self.goal_force = float(goal_handle.request.force)
        feedback = Grasp.Feedback()
        result = Grasp.Result()
        self.gripper.set_twist_size(self.goal_width)

        self.gripper.update()

        feedback.current_width = float(self.gripper.twist_size)
        feedback.current_force = 0.0
        goal_handle.publish_feedback(feedback)

        while self.gripper.twist_status == 3:
            self.gripper.update()

            feedback.current_width = float(self.gripper.twist_size)
            feedback.current_force = 0.0
            goal_handle.publish_feedback(feedback)

        if self.gripper.twist_status == 1:
            result.success = True
            result.error = ""
            goal_handle.succeed()
        elif self.gripper.twist_status == 2:
            result.success = False
            result.error = "Loop is homing"
            goal_handle.abort()
        elif self.gripper.twist_status == 4:
            result.success = False
            result.error = "Loop hit the endstop"
            goal_handle.abort()
        elif self.gripper.twist_status == 5:
            result.success = True
            result.error = ""
            goal_handle.abort()

        return result

    def twist_twist_callback(self, goal_handle):
        self.gripper.enable(self.gripper.TWIST)
        self.gripper.switch_to_position_control(self.gripper.TWIST)
        self.goal = float(goal_handle.request.target)
        feedback = Twist.Feedback()
        result = Twist.Result()
        self.gripper.set_twist(self.goal)

        self.gripper.update()

        feedback.current_offset = float(self.gripper.twist_offset)
        goal_handle.publish_feedback(feedback)

        while self.gripper.twist_status == 3:
            self.gripper.update()

            feedback.current_offset = float(self.gripper.twist_offset)
            goal_handle.publish_feedback(feedback)

        if self.gripper.twist_status == 1:
            result.success = True
            result.error = ""
            goal_handle.succeed()
        elif self.gripper.twist_status == 2:
            result.success = False
            result.error = "Twist is homing"
            goal_handle.abort()
        elif self.gripper.twist_status == 4:
            result.success = False
            result.error = "Twist hit the endstop"
            goal_handle.abort()
        elif self.gripper.twist_status == 5:
            result.success = False
            result.error = "Twist is stuck"
            goal_handle.abort()

        return result

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
