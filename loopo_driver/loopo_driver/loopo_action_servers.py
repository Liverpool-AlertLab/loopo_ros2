import rclpy

from loopo_driver import LoopODriver

from rclpy.action import ActionServer
from rclpy.node import Node

from loopo_messages.action import Move, Homing, Grasp, Twist


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

    def debug_tw(self):
        self.get_logger().info(
            "status: %d, control: %d, size: %f, offeset: %f"
            % (
                self.gripper.tw_status,
                self.gripper.tw_control,
                self.gripper.tw_size,
                self.gripper.tw_offset,
            )
        )

    def debug_lp(self):
        self.get_logger().info(
            "status: %d, control: %d, size: %f, force: %f"
            % (
                self.gripper.lp_status,
                self.gripper.lp_control,
                self.gripper.lp_size,
                self.gripper.force,
            )
        )

    def extension_is_position_ready(self):
        self.gripper.send_command(0, 0, 0.0)
        if self.gripper.ex_status == 0:
            self.gripper.send_command(1, 0, 1.0)
        if self.gripper.ex_control != 1:
            self.gripper.send_command(1, 1, 1.0)

    def extension_move_callback(self, goal_handle):
        self.extension_is_position_ready()
        self.goal = float(goal_handle.request.target)
        feedback = Move.Feedback()
        result = Move.Result()
        self.gripper.send_command(1, 3, self.goal)
        self.gripper.send_command(0, 0, 0.0)

        feedback.current_position = float(self.gripper.ex_position)
        goal_handle.publish_feedback(feedback)

        while self.gripper.ex_status == 3:
            self.gripper.send_command(0, 0, 0.0)

            feedback.current_position = float(self.gripper.ex_position)
            goal_handle.publish_feedback(feedback)

        if self.gripper.ex_status == 1:
            result.success = True
            result.error = ""
            goal_handle.succeed()
        elif self.gripper.ex_status == 2:
            result.success = False
            result.error = "Extension is homing"
            goal_handle.abort()
        elif self.gripper.ex_status == 4:
            result.success = False
            result.error = "Extension hit the endstop"
            goal_handle.abort()
        elif self.gripper.ex_status == 5:
            result.success = False
            result.error = "Extension is stuck"
            goal_handle.abort()

        return result

    def extension_home_callback(self, goal_handle):
        self.goal = float(goal_handle.request.speed)
        feedback = Homing.Feedback()
        result = Homing.Result()
        self.gripper.send_command(1, 5, self.goal)
        self.gripper.send_command(0, 0, 0.0)
        feedback.current_position = float(self.gripper.ex_position)
        goal_handle.publish_feedback(feedback)

        while self.gripper.ex_status == 2:
            self.gripper.send_command(0, 0, 0.0)
            feedback.current_position = float(self.gripper.ex_position)
            goal_handle.publish_feedback(feedback)

        if self.gripper.ex_status == 1:
            result.success = True
            result.error = ""
            goal_handle.succeed()
        elif self.gripper.ex_status == 4:
            result.success = True
            result.error = "Extension hit the endstop"
            goal_handle.succeed()
        elif self.gripper.ex_status == 5:
            result.success = False
            result.error = "Extension is stuck"
            goal_handle.abort()

        return result

    def twist_is_position_ready(self):
        self.gripper.send_command(0, 0, 0.0)
        if self.gripper.tw_status == 0:
            self.gripper.send_command(2, 0, 1.0)
        if self.gripper.tw_control != 1:
            self.gripper.send_command(2, 1, 1.0)

    def twist_move_callback(self, goal_handle):
        self.twist_is_position_ready()
        self.goal = float(goal_handle.request.target)
        feedback = Move.Feedback()
        result = Move.Result()
        self.gripper.send_command(2, 3, self.goal)
        self.gripper.send_command(0, 0, 0.0)

        feedback.current_position = float(self.gripper.tw_size)
        goal_handle.publish_feedback(feedback)

        while self.gripper.tw_status == 3:
            self.gripper.send_command(0, 0, 0.0)

            feedback.current_position = float(self.gripper.tw_size)
            goal_handle.publish_feedback(feedback)

        if self.gripper.tw_status == 1:
            result.success = True
            result.error = ""
            goal_handle.succeed()
        elif self.gripper.tw_status == 2:
            result.success = False
            result.error = "Twist is homing"
            goal_handle.abort()
        elif self.gripper.tw_status == 4:
            result.success = False
            result.error = "Twist hit the endstop"
            goal_handle.abort()
        elif self.gripper.tw_status == 5:
            result.success = False
            result.error = "Twist is stuck"
            goal_handle.abort()

        return result

    def twist_home_callback(self, goal_handle):
        self.goal = float(goal_handle.request.speed)
        feedback = Homing.Feedback()
        result = Homing.Result()
        self.gripper.send_command(2, 5, self.goal)
        feedback.current_position = float(self.gripper.tw_size)
        goal_handle.publish_feedback(feedback)
        self.gripper.send_command(0, 0, 0)

        while self.gripper.tw_status == 2:
            self.gripper.send_command(0, 0, 0.0)
            feedback.current_position = float(self.gripper.tw_size)
            goal_handle.publish_feedback(feedback)

        if self.gripper.tw_status == 1:
            result.success = True
            result.error = ""
            goal_handle.succeed()
        elif self.gripper.tw_status == 4:
            result.success = True
            result.error = "Twist hit the endstop"
            goal_handle.succeed()
        elif self.gripper.tw_status == 5:
            result.success = False
            result.error = "Twist is stuck"
            goal_handle.abort()

        return result

    def twist_grasp_callback(self, goal_handle):
        self.loop_is_position_ready()
        self.goal_width = float(goal_handle.request.width)
        self.goal_force = float(goal_handle.request.force)
        feedback = Grasp.Feedback()
        result = Grasp.Result()
        self.gripper.send_command(2, 3, self.goal_width)

        self.gripper.send_command(0, 0, 0.0)

        feedback.current_width = float(self.gripper.tw_size)
        feedback.current_force = 0.0
        goal_handle.publish_feedback(feedback)

        while self.gripper.tw_status == 3:
            self.gripper.send_command(0, 0, 0.0)

            feedback.current_width = float(self.gripper.tw_size)
            feedback.current_force = 0.0
            goal_handle.publish_feedback(feedback)

        if self.gripper.tw_status == 1:
            result.success = True
            result.error = ""
            goal_handle.succeed()
        elif self.gripper.tw_status == 2:
            result.success = False
            result.error = "Loop is homing"
            goal_handle.abort()
        elif self.gripper.tw_status == 4:
            result.success = False
            result.error = "Loop hit the endstop"
            goal_handle.abort()
        elif self.gripper.tw_status == 5:
            result.success = True
            result.error = ""
            goal_handle.abort()

        return result

    def twist_twist_callback(self, goal_handle):
        self.twist_is_position_ready()
        self.goal = float(goal_handle.request.target)
        feedback = Twist.Feedback()
        result = Twist.Result()
        self.gripper.send_command(2, 6, self.goal)

        self.gripper.send_command(0, 0, 0.0)

        feedback.current_offset = float(self.gripper.tw_offset)
        goal_handle.publish_feedback(feedback)

        while self.gripper.tw_status == 3:
            self.gripper.send_command(0, 0, 0.0)

            feedback.current_offset = float(self.gripper.tw_offset)
            goal_handle.publish_feedback(feedback)

        if self.gripper.tw_status == 1:
            result.success = True
            result.error = ""
            goal_handle.succeed()
        elif self.gripper.tw_status == 2:
            result.success = False
            result.error = "Twist is homing"
            goal_handle.abort()
        elif self.gripper.tw_status == 4:
            result.success = False
            result.error = "Twist hit the endstop"
            goal_handle.abort()
        elif self.gripper.tw_status == 5:
            result.success = False
            result.error = "Twist is stuck"
            goal_handle.abort()

        return result

    def loop_is_position_ready(self):
        self.gripper.send_command(0, 0, 0.0)
        if self.gripper.lp_status == 0:
            self.gripper.send_command(3, 0, 1.0)
        if self.gripper.lp_control != 1:
            self.gripper.send_command(3, 1, 1.0)

    def loop_is_force_ready(self):
        self.gripper.send_command(0, 0, 0.0)
        if self.gripper.lp_status == 0:
            self.gripper.send_command(3, 0, 1.0)
        if self.gripper.lp_control != 1:
            self.gripper.send_command(3, 1, 3.0)

    def loop_move_callback(self, goal_handle):
        self.loop_is_position_ready()
        self.goal = float(goal_handle.request.target)
        feedback = Move.Feedback()
        result = Move.Result()
        self.gripper.send_command(3, 3, self.goal)
        self.gripper.send_command(0, 0, 0.0)
        feedback.current_position = float(self.gripper.lp_size)
        goal_handle.publish_feedback(feedback)

        while self.gripper.lp_status == 3:
            self.gripper.send_command(0, 0, 0.0)
            feedback.current_position = float(self.gripper.lp_size)
            goal_handle.publish_feedback(feedback)

        if self.gripper.lp_status == 1:
            result.success = True
            result.error = ""
            goal_handle.succeed()
        elif self.gripper.lp_status == 2:
            result.success = False
            result.error = "Loop is homing"
            goal_handle.abort()
        elif self.gripper.lp_status == 4:
            result.success = False
            result.error = "Loop hit the endstop"
            goal_handle.abort()
        elif self.gripper.lp_status == 5:
            result.success = False
            result.error = "Loop is stuck"
            self.get_logger().info("Error: %s" % (result.error))
            goal_handle.abort()

        return result

    def loop_home_callback(self, goal_handle):
        self.goal = float(goal_handle.request.speed)
        feedback = Homing.Feedback()
        result = Homing.Result()
        self.gripper.send_command(3, 5, self.goal)
        feedback.current_position = float(self.gripper.lp_size)
        goal_handle.publish_feedback(feedback)

        while self.gripper.lp_status == 2:
            self.gripper.send_command(0, 0, 0.0)
            goal_handle.publish_feedback(feedback)

        if self.gripper.lp_status == 1:
            result.success = True
            result.error = ""
            goal_handle.succeed()
        elif self.gripper.lp_status == 4:
            result.success = True
            result.error = "Loop hit the endstop"
            goal_handle.succeed()
        elif self.gripper.lp_status == 5:
            result.success = False
            result.error = "Loop is stuck"
            goal_handle.abort()

        return result

    def loop_grasp_callback(self, goal_handle):
        self.loop_is_position_ready()
        self.goal_width = float(goal_handle.request.width)
        self.goal_force = float(goal_handle.request.force)
        feedback = Grasp.Feedback()
        result = Grasp.Result()
        self.gripper.send_command(3, 3, self.goal_width)
        self.gripper.send_command(0, 0, 0.0)
        feedback.current_width = float(self.gripper.lp_size)
        feedback.current_force = float(self.gripper.force)
        goal_handle.publish_feedback(feedback)

        while self.gripper.lp_status == 3:
            self.gripper.send_command(0, 0, 0.0)
            feedback.current_width = float(self.gripper.lp_size)
            feedback.current_force = float(self.gripper.force)
            goal_handle.publish_feedback(feedback)

        if self.gripper.lp_status == 2:
            result.success = False
            result.error = "Loop is homing"
            goal_handle.abort()
        elif self.gripper.lp_status == 4:
            result.success = False
            result.error = "Loop hit the endstop"
            goal_handle.abort()

        self.gripper.send_comand(3, 6, self.goal_force)
        self.loop_is_force_ready()

        while self.gripper.lp_status != 6:
            self.gripper.send_command(0, 0, 0.0)
            feedback.current_width = float(self.gripper.lp_size)
            feedback.current_force = float(self.gripper.force)
            goal_handle.publish_feedback(feedback)

        result.success = True
        result.error = "Loop is stuck"
        self.get_logger().info("Error: %s" % (result.error))
        goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)

    loopo = LoopODriver(com_port="/dev/ttyACM1")

    extension_action_server = LoopOActionServers(loopo_driver=loopo)

    rclpy.spin(extension_action_server)


if __name__ == "__main__":
    main()
