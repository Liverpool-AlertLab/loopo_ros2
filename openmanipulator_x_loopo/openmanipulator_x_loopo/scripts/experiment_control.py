from math import exp
import rclpy

from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState

import time


class Manipulator(Node):
    qos = QoSProfile(depth=10)

    def __init__(self):
        super().__init__("manipulator")
        key_value = ""

        # Create joint_states subscriber
        self.joint_subscription = self.create_subscription(
            JointState, "joint_states", self.joint_state_callback, self.qos
        )
        # Create kinematics_pose subscriber
        self.pose_subscription = self.create_subscription(
            KinematicsPose, "kinematics_pose", self.kinematics_pose_callback, self.qos
        )

        # Create manipulator state subscriber
        self.state_subscription = self.create_subscription(
            OpenManipulatorState,
            "states",
            self.open_manipulator_state_callback,
            self.qos,
        )

        # Create Service Clients
        self.goal_joint_space = self.create_client(
            SetJointPosition, "goal_joint_space_path"
        )
        self.goal_task_space = self.create_client(
            SetKinematicsPose, "goal_task_space_path"
        )
        self.tool_control = self.create_client(SetJointPosition, "goal_tool_control")

        self.goal_joint_req = SetJointPosition.Request()
        self.goal_pose_req = SetKinematicsPose.Request()
        self.tool_control_req = SetJointPosition.Request()

        self.manipulator_moving_state = "STOPPED"
        self.present_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.present_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]

    def send_goal_pose(
        self,
        position=[0.14, 0.0, 0.8],
        orientation=[0.714, 0.0, 0.714, 0.0],
        path_time=0.5,
    ):
        self.goal_pose_req.end_effector_name = "gripper"
        self.goal_pose_req.kinematics_pose.pose.position.x = position[0]
        self.goal_pose_req.kinematics_pose.pose.position.y = position[1]
        self.goal_pose_req.kinematics_pose.pose.position.z = position[2]
        self.goal_pose_req.kinematics_pose.pose.orientation.w = orientation[0]
        self.goal_pose_req.kinematics_pose.pose.orientation.x = orientation[1]
        self.goal_pose_req.kinematics_pose.pose.orientation.y = orientation[2]
        self.goal_pose_req.kinematics_pose.pose.orientation.z = orientation[3]
        self.goal_pose_req.path_time = path_time

        try:
            self.future_goal_pose = self.goal_task_space.call_async(self.goal_pose_req)
        except Exception as e:
            self.get_logger().info("Sending Goal Kinematic Pose failed %r" % (e,))

    def send_goal_joint(self, angles=[0.0, 0.0, 0.0, 1.57, 0.0], path_time=1.0):
        self.goal_joint_req.joint_position.joint_name = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "gripper",
        ]
        self.goal_joint_req.joint_position.position = [
            angles[0],
            angles[1],
            angles[2],
            angles[3],
            angles[4],
        ]
        self.goal_joint_req.path_time = path_time

        try:
            self.future_joint_goal = self.goal_joint_space.call_async(
                self.goal_joint_req
            )
        except Exception as e:
            self.get_logger().info("Sending Goal Joint failed %r" % (e,))

    def send_tool_control_request(
        self, angles=[0.0, 0.0, 0.0, 1.57, 0.0], path_time=1.0
    ):
        self.tool_control_req.joint_position.joint_name = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "gripper",
        ]
        self.tool_control_req.joint_position.position = [
            angles[0],
            angles[1],
            angles[2],
            angles[3],
            angles[4],
        ]
        self.tool_control_req.path_time = path_time

        try:
            self.future_tool_control = self.tool_control.call_async(
                self.tool_control_req
            )

        except Exception as e:
            self.get_logger().info("Tool control failed %r" % (e,))

    def kinematics_pose_callback(self, msg):
        self.present_pose[0] = msg.pose.position.x
        self.present_pose[1] = msg.pose.position.y
        self.present_pose[2] = msg.pose.position.z
        self.present_pose[3] = msg.pose.orientation.w
        self.present_pose[4] = msg.pose.orientation.x
        self.present_pose[5] = msg.pose.orientation.y
        self.present_pose[6] = msg.pose.orientation.z

    def joint_state_callback(self, msg):
        self.present_joint_angle[0] = msg.position[0]
        self.present_joint_angle[1] = msg.position[1]
        self.present_joint_angle[2] = msg.position[2]
        self.present_joint_angle[3] = msg.position[3]
        self.present_joint_angle[4] = msg.position[4]

    def open_manipulator_state_callback(self, msg):
        self.manipulator_moving_state = msg.open_manipulator_moving_state


try:
    rclpy.init()
except Exception as e:
    print(e)

try:
    manipulator = Manipulator()
except Exception as e:
    print(e)


def wait_for_movement():
    while manipulator.manipulator_moving_state == "STOPPED":
        rclpy.spin_once(manipulator)
    while manipulator.manipulator_moving_state == "IS_MOVING":
        rclpy.spin_once(manipulator)


def move2xyz(pos=[0.0, 0.1, 0.05], path_time=1.0):
    manipulator.send_goal_pose(position=pos, path_time=path_time)
    rclpy.spin_until_future_complete(manipulator, manipulator.future_goal_pose)
    time.sleep(path_time)


def main():
    try:
        manipulator.send_goal_joint([0.0, 0.0, 0.0, 1.57, 0.05], 3.0)
        rclpy.spin_until_future_complete(manipulator, manipulator.future_joint_goal)

        manipulator.get_logger().info("joint command sent")

        wait_for_movement()

        manipulator.get_logger().info("pose command sent")
        move2xyz([0.1, 0.1, 0.1], 1.0)

        manipulator.get_logger().info("pose command sent")
        move2xyz([0.1, 0.1, 0.05], 1.0)

        manipulator.send_tool_control_request([0.0, 0.0, 0.0, 1.57, 0.01], 1.0)
        rclpy.spin_until_future_complete(manipulator, manipulator.future_joint_goal)
        time.sleep(1.0)

        manipulator.get_logger().info("tool command sent")

        while manipulator.manipulator_moving_state == "IS_MOVING":
            rclpy.spin_once(manipulator)

        manipulator.get_logger().info("pose command sent")
        move2xyz([0.2, 0.0, 0.0], 1.0)

        manipulator.send_tool_control_request([0.0, 0.0, 0.0, 1.57, -0.005], 1.0)
        rclpy.spin_until_future_complete(manipulator, manipulator.future_joint_goal)
        manipulator.get_logger().info("tool command sent")
        time.sleep(1.0)

        move2xyz([0.2, 0.0, -0.02])

        manipulator.get_logger().info("finished")

    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
