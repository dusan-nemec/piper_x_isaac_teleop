"""PiPER-X + gripper in Isaac Sim: ros2_control, MoveIt, Servo, joystick teleop.

Reuses piper_x_gripper_moveit_config unchanged. The only modification is done
in memory: its ros2_control hardware (mock_components/GenericSystem) is swapped
for topic_based_ros2_control, so controllers talk to Isaac Sim via topics.

    ros2_control  --/ros_joint_commands-->  isaac_bridge  --/isaac_joint_commands-->  Isaac Sim
    ros2_control  <--/ros_joint_states---   isaac_bridge  <--/isaac_joint_states----  Isaac Sim
"""
import os
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

THIS_PKG = "piper_x_isaac_teleop"

ARGS = [
    ("moveit_config_pkg", "piper_x_gripper_moveit_config", "MoveIt config package to reuse"),
    ("robot_name", "piper_x", "Name used by the MoveIt config files (config/<name>.srdf ...)"),
    ("move_group", "arm", "Planning group Servo controls"),
    ("ee_frame", "link6", "End-effector frame for Servo (e.g. tcp_link if your URDF has one)"),
    ("arm_controller", "arm_controller", "JointTrajectoryController for the arm"),
    ("gripper_controller", "gripper_controller", "JointTrajectoryController for the gripper"),
    ("gripper_joint", "gripper", "Virtual gripper joint on the ROS side"),
    ("joy_dev", "0", "Joystick device id for joy_node"),
    ("use_rviz", "true", "Start RViz"),
    ("use_joy", "true", "Start joy_node + joy_teleop"),
]

HW_PLUGIN = (
    "<plugin>topic_based_ros2_control/TopicBasedSystem</plugin>"
    '<param name="joint_commands_topic">/ros_joint_commands</param>'
    '<param name="joint_states_topic">/ros_joint_states</param>'
)


def launch_setup(context, *_):
    arg = lambda n: LaunchConfiguration(n).perform(context)
    sim = {"use_sim_time": True}
    moveit_pkg = arg("moveit_config_pkg")
    moveit_share = get_package_share_directory(moveit_pkg)

    mc = MoveItConfigsBuilder(arg("robot_name"), package_name=moveit_pkg).to_moveit_configs()

    # --- robot_description with Isaac (topic-based) hardware -----------------
    urdf = mc.robot_description["robot_description"]
    if not isinstance(urdf, str):
        raise RuntimeError("robot_description is not a plain string; cannot patch hardware plugin")
    urdf, n = re.subn(r"<plugin>\s*mock_components/GenericSystem\s*</plugin>", HW_PLUGIN, urdf)
    if n == 0:
        raise RuntimeError(f"No mock_components/GenericSystem found in {moveit_pkg}'s URDF")
    robot_description = {"robot_description": urdf}

    controllers_yaml = os.path.join(moveit_share, "config", "ros2_controllers.yaml")

    # --- Servo parameters -----------------------------------------------------
    with open(os.path.join(get_package_share_directory(THIS_PKG), "config", "servo.yaml")) as f:
        servo = yaml.safe_load(f)
    servo["move_group_name"] = arg("move_group")
    servo["ee_frame_name"] = servo["ee_frame"] = arg("ee_frame")
    servo["command_out_topic"] = f"/{arg('arm_controller')}/joint_trajectory"
    servo_exe = "servo_node_main" if os.environ.get("ROS_DISTRO") == "humble" else "servo_node"

    spawner = lambda name: Node(
        package="controller_manager", executable="spawner",
        arguments=[name, "--controller-manager", "/controller_manager"],
        parameters=[sim], output="screen")

    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[robot_description, sim], output="screen"),

        Node(package=THIS_PKG, executable="isaac_bridge",
             parameters=[{"gripper_joint": arg("gripper_joint")}, sim], output="screen"),

        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[robot_description, controllers_yaml, sim],
             remappings=[("~/robot_description", "/robot_description")],  # Jazzy
             output="screen"),

        # give the controller manager a moment, then spawn controllers
        TimerAction(period=2.0, actions=[
            spawner("joint_state_broadcaster"),
            spawner(arg("arm_controller")),
            spawner(arg("gripper_controller")),
        ]),

        Node(package="moveit_ros_move_group", executable="move_group",
             parameters=[mc.to_dict(), sim], output="screen"),

        Node(package="moveit_servo", executable=servo_exe, name="servo_node",
             parameters=[{"moveit_servo": servo},
                         mc.robot_description, mc.robot_description_semantic,
                         mc.robot_description_kinematics, mc.joint_limits, sim],
             output="screen"),

        Node(package="rviz2", executable="rviz2",
             arguments=["-d", os.path.join(moveit_share, "config", "moveit.rviz")],
             parameters=[mc.robot_description, mc.robot_description_semantic,
                         mc.robot_description_kinematics, mc.planning_pipelines,
                         mc.joint_limits, sim],
             condition=IfCondition(arg("use_rviz")), output="log"),

        Node(package="joy", executable="joy_node",
             parameters=[{"device_id": int(arg("joy_dev")), "autorepeat_rate": 50.0,
                          "deadzone": 0.05}, sim],
             condition=IfCondition(arg("use_joy")), output="screen"),

        Node(package=THIS_PKG, executable="joy_teleop",
             parameters=[{"gripper_controller": arg("gripper_controller"),
                          "gripper_joint": arg("gripper_joint")}, sim],
             condition=IfCondition(arg("use_joy")), output="screen"),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in ARGS]
        + [OpaqueFunction(function=launch_setup)]
    )
