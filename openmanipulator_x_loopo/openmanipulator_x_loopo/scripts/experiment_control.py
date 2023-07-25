from math import exp
import rclpy

from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

present_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
present_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class Experiment(Node):
    qos = QoSProfile(depth=10)

    def __init__(self):
        super().__init__("experiment")
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

    def send_goal_joint(self, path_time):
        self.goal_joint_req.joint_position.joint_name = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "gripper",
        ]
        self.goal_joint_req.joint_position.position = [
            goal_joint_angle[0],
            goal_joint_angle[1],
            goal_joint_angle[2],
            goal_joint_angle[3],
            goal_joint_angle[4],
        ]
        self.goal_joint_req.path_time = path_time

        try:
            self.future_joint_goal = self.goal_joint_space.call_async(
                self.goal_joint_req
            )
        except Exception as e:
            self.get_logger().info("Sending Goal Joint failed %r" % (e,))

    def send_tool_control_request(self, path_time):
        self.tool_control_req.joint_position.joint_name = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "gripper",
        ]
        self.tool_control_req.joint_position.position = [
            goal_joint_angle[0],
            goal_joint_angle[1],
            goal_joint_angle[2],
            goal_joint_angle[3],
            goal_joint_angle[4],
        ]
        self.tool_control_req.path_time = path_time

        try:
            self.future_tool_control = self.tool_control.call_async(
                self.tool_control_req
            )

        except Exception as e:
            self.get_logger().info("Tool control failed %r" % (e,))

    def kinematics_pose_callback(self, msg):
        present_kinematics_pose[0] = msg.pose.position.x
        present_kinematics_pose[1] = msg.pose.position.y
        present_kinematics_pose[2] = msg.pose.position.z
        present_kinematics_pose[3] = msg.pose.orientation.w
        present_kinematics_pose[4] = msg.pose.orientation.x
        present_kinematics_pose[5] = msg.pose.orientation.y
        present_kinematics_pose[6] = msg.pose.orientation.z

    def joint_state_callback(self, msg):
        present_joint_angle[0] = msg.position[0]
        present_joint_angle[1] = msg.position[1]
        present_joint_angle[2] = msg.position[2]
        present_joint_angle[3] = msg.position[3]
        present_joint_angle[4] = msg.position[4]

    def open_manipulator_state_callback(self, msg):
        self.manipulator_moving_state = msg.open_manipulator_moving_state
        if msg.open_manipulator_moving_state == "STOPPED":
            for index in range(0, 7):
                goal_kinematics_pose[index] = present_kinematics_pose[index]
            for index in range(0, 5):
                goal_joint_angle[index] = present_joint_angle[index]

    def print_present_values():
        print(
            "Joint Angle(Rad): [{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}]".format(
                present_joint_angle[0],
                present_joint_angle[1],
                present_joint_angle[2],
                present_joint_angle[3],
                present_joint_angle[4],
            )
        )
        print(
            "Kinematics Pose(Pose X, Y, Z | Orientation W, X, Y, Z): {:.3f}, {:.3f}, {:.3f} | {:.3f}, {:.3f}, {:.3f}, {:.3f}".format(
                present_kinematics_pose[0],
                present_kinematics_pose[1],
                present_kinematics_pose[2],
                present_kinematics_pose[3],
                present_kinematics_pose[4],
                present_kinematics_pose[5],
                present_kinematics_pose[6],
            )
        )


def main():
    try:
        rclpy.init()
    except Exception as e:
        print(e)

    try:
        experiment = Experiment()
    except Exception as e:
        print(e)

    counter = 0

    try:
        goal_joint_angle[0] = 0.0
        goal_joint_angle[1] = 0.0
        goal_joint_angle[2] = 0.0
        goal_joint_angle[3] = 1.57

        experiment.send_goal_joint(3.0)
        rclpy.spin_until_future_complete(experiment, experiment.future_joint_goal)

        experiment.get_logger().info("joint command sent")

        while experiment.manipulator_moving_state == "STOPPED":
            rclpy.spin_once(experiment)
        while experiment.manipulator_moving_state == "IS_MOVING":
            rclpy.spin_once(experiment)

        experiment.get_logger().info("pose command sent")
        experiment.send_goal_pose([0.1, 0.1, 0.1], path_time=1.0)
        rclpy.spin_until_future_complete(experiment, experiment.future_goal_pose)

        while experiment.manipulator_moving_state == "STOPPED":
            rclpy.spin_once(experiment)
        while experiment.manipulator_moving_state == "IS_MOVING":
            rclpy.spin_once(experiment)

        experiment.get_logger().info("pose command sent")
        experiment.send_goal_pose([0.1, 0.1, -0.05], path_time=1.0)
        rclpy.spin_until_future_complete(experiment, experiment.future_goal_pose)

        while experiment.manipulator_moving_state == "STOPPED":
            rclpy.spin_once(experiment)
        while experiment.manipulator_moving_state == "IS_MOVING":
            rclpy.spin_once(experiment)

        experiment.get_logger().info("pose command sent")
        experiment.send_goal_pose([0.2, 0.0, 0.0], path_time=1.0)
        rclpy.spin_until_future_complete(experiment, experiment.future_goal_pose)

        while experiment.manipulator_moving_state == "STOPPED":
            rclpy.spin_once(experiment)
        while experiment.manipulator_moving_state == "IS_MOVING":
            rclpy.spin_once(experiment)

        experiment.get_logger().info("finished")

    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
