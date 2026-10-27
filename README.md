# piper_x_isaac_teleop

Runs the AgileX PiPER-X (with gripper) in Isaac Sim through ros2_control, MoveIt 2
and MoveIt Servo, with Cartesian joystick teleoperation. It reuses an existing MoveIt config
without modifying it: by default AgileX's official `agx_arm_moveit`
(from [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros)), or any
MoveIt Setup Assistant package such as `piper_x_gripper_moveit_config`.

```
joy_node -> joy_teleop -> servo_node -> arm_controller (JTC)
                     \--------------> gripper_controller (JTC)
                                              |
                       ros2_control (topic_based_ros2_control)
                         /ros_joint_commands      /ros_joint_states
                                  |                      ^
                              isaac_bridge  (gripper <-> 2 fingers)
                                  v                      |
                         /isaac_joint_commands   /isaac_joint_states
                                        Isaac Sim
```

## What it does

* Loads the URDF, SRDF, kinematics, joint limits, controllers and RViz config
  from the MoveIt config package. For `agx_arm_moveit` it uses the same xacro
  mappings as AgileX's own launch files (`arm_type:=piper_x`,
  `effector_type:=agx_gripper`, `tcp_offset`).
* Replaces whatever ros2_control hardware the URDF declares (mock, real-robot
  driver, Gazebo...) with `topic_based_ros2_control`, in memory at launch.
  If the URDF has no `<ros2_control>` block, one is added for the joints in
  the `ros2_control_joints` launch argument. Your MoveIt config package stays
  untouched.
* `isaac_bridge` maps the ROS-side virtual `gripper` joint (opening width,
  0-0.1 m) to the two Isaac finger joints (`gripper_joint1 = w/2`,
  `gripper_joint2 = -w/2`) and back.
* Servo streams arm trajectories to `arm_controller` at 100 Hz; buttons send
  gripper goals to `gripper_controller`.

## Requirements

* MoveIt 2 + `moveit_servo`, `ros2_control`, `ros2_controllers`, `joy`
* [`topic_based_ros2_control`](https://github.com/PickNikRobotics/topic_based_ros2_control)
  (build from source if it isn't packaged for your distro)
* `agx_arm_ros` (for `agx_arm_moveit` and `agx_arm_description`) in the same
  workspace, or your own MoveIt config package. Controllers are expected to be
  named `arm_controller` and `gripper_controller`
  (`joint_trajectory_controller/JointTrajectoryController`); other names can
  be passed as launch args.

## Isaac Sim side

The Action Graph must use these topics (rename them in the graph):

| Node | Topic |
|---|---|
| ROS2 Publish Joint State | `isaac_joint_states` |
| ROS2 Subscribe Joint State | `isaac_joint_commands` |
| ROS2 Publish Clock | `clock` |

## Build and run

```bash
cd ~/ws && colcon build --symlink-install --packages-select piper_x_isaac_teleop
source install/setup.bash

# 1. Isaac Sim: open the stage and press Play
# 2. then:
ros2 launch piper_x_isaac_teleop bringup.launch.py
```

Useful launch arguments:

| Argument | Default | Notes |
|---|---|---|
| `moveit_config_pkg` | `agx_arm_moveit` | or e.g. `piper_x_gripper_moveit_config` |
| `arm_type` | `piper_x` | `agx_arm_moveit` only |
| `tcp_offset` | `[0,0,0,0,0,0]` | `agx_arm_moveit` only: TCP offset from the flange |
| `robot_name` | `piper_x` | Setup-Assistant packages only: `config/<robot_name>.srdf` |
| `move_group` | `arm` | planning group from the SRDF |
| `ee_frame` | auto | `tcp_link` for `agx_arm_moveit`, `link6` otherwise |
| `joint_limits` | auto | `agx_arm_moveit`'s own file; this package's file for Setup-Assistant packages |
| `arm_controller` / `gripper_controller` | `arm_controller` / `gripper_controller` | names from `ros2_controllers.yaml` |
| `use_rviz`, `use_joy` | `true` | |

Joystick mapping (Xbox pad on Linux): left stick = X/Y, right stick = Z/yaw,
LB/RB = roll, A/B = open/close gripper. Change the axes with `joy_teleop`
parameters (`axis_x`, `axis_y`, `axis_z`, `axis_yaw`, `button_*`).

## Notes

* Move the arm out of the zero pose before teleoperating. At zero, joints 2 and 3
  sit on their limits and Servo will refuse or crawl. Plan to a bent pose in RViz first.
* Planning in RViz and Servo both use `arm_controller`. Don't do both at once.
* Humble and Jazzy are both handled: the launch picks `servo_node_main` or
  `servo_node`, and `joy_teleop` calls `start_servo` (Humble) or
  `switch_command_type` (Jazzy).
