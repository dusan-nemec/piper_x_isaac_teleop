# piper_x_isaac_teleop

Runs the AgileX PiPER-X (with gripper) in Isaac Sim through ros2_control, MoveIt 2
and MoveIt Servo, with Cartesian joystick teleoperation. It reuses an existing MoveIt config
without modifying it: by default AgileX's official `agx_arm_moveit`
(from [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros)), or any
MoveIt Setup Assistant package such as `piper_x_gripper_moveit_config`.

```
joy_node -> joy_teleop -> robot_supervisor -> servo_node -> arm_controller (JTC)
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

Joystick mapping (Xbox pad on Linux):

| Input | Action |
|---|---|
| left stick up/down | linear X velocity |
| left stick left/right | linear Y velocity |
| right stick up/down | linear Z velocity |
| hold Y + sticks | angular Y / X / Z velocity |
| Back / X | joint move to `home` / `tool_grab` |
| Start | abort a running joint move |
| A / B | close / open gripper |

### Robot supervisor

`supervisor` (node `robot_supervisor`) wraps MoveIt Servo and move_group in simple
topics. It is the only node that sends motion commands:

| Topic | Type | Motion |
|---|---|---|
| `/supervisor/linear_target` | `geometry_msgs/Pose` | straight-line EE move (Servo pose tracking) |
| `/supervisor/joint_target` | `geometry_msgs/Pose` | planned move to an EE pose: `MoveGroup` action (`/move_action`), collision-free path |
| `/supervisor/joint_command` | `sensor_msgs/JointState` | planned move to joint values: `MoveGroup` action (`/move_action`), collision-free path |
| `/supervisor/velocity_target` | `geometry_msgs/Twist` | Cartesian velocity (Servo twist); a zero Twist stops any motion |
| `/supervisor/state` (out) | `std_msgs/String` | `done`, `busy` or `failed` (on change) |

Poses are `ee_frame` poses in `planning_frame` (`base_link`). Twists are unitless
(+-1, scaled by Servo's `scale.linear` / `scale.rotational`) and must keep coming:
motion stops `velocity_timeout` (0.2 s) after the last one. A new target replaces
the running one. Planned moves (`joint_target`, `joint_command`) are planned and
executed by move_group. The path avoids collisions but is not a straight line. The
goal tolerance is `linear_tolerance` / `angular_tolerance` for poses and
`joint_tolerance` for joint values; speed is set by `velocity_scaling` /
`acceleration_scaling`, and planning by `planning_time` / `planning_attempts`.
A linear move first checks the whole straight segment against the planning scene
(collision objects, octomap) with move_group's `/compute_cartesian_path`, in
`path_check_step` (5 mm) steps, and fails without moving if the segment is blocked.
While moving, Servo's collision checking slows down near obstacles, and a Servo halt
for collision ends the move as `failed`. A stream of linear targets is checked once
at the start; after that only Servo guards it.
A linear move ends as `done` when the EE is within `linear_tolerance` /
`angular_tolerance`; a planned move ends with move_group's result. A move ends as
`failed` on a planning or execution error, after `move_timeout` (30 s), or when
stopped by a zero Twist.

```bash
ros2 topic pub --once /supervisor/joint_target geometry_msgs/msg/Pose \
  "{position: {x: 0.25, y: 0.0, z: 0.25}, orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}}"
ros2 topic echo /supervisor/state
```

### Joystick

`joy_teleop` turns the joystick into supervisor targets. The sticks send Cartesian
velocity on `/supervisor/velocity_target` (unitless, `base_link` axes), and releasing
them sends a zero Twist, which stops the arm.

Predefined poses live in [`config/poses.yaml`](config/poses.yaml), each with an
optional joystick `button`:

* `joints`: sent to `/supervisor/joint_command`. move_group plans a collision-free
  path and executes it on `arm_controller` (supervisor parameters `planning_time`,
  `velocity_scaling`, `acceleration_scaling`).
* `pose` ([x y z r p y] or [x y z qx qy qz qw]): sent to `/supervisor/joint_target`. During such a joint move the sticks are ignored and Start aborts it.
Trigger a pose by name:

```bash
ros2 topic pub --once /joy_teleop/goto std_msgs/msg/String "{data: home}"
```

Remap the sticks and buttons with `joy_teleop` parameters `x`, `y`, `z` (axes) and
`open`, `close`, `attitude`, `abort` (buttons).

## Notes

* Move the arm out of the zero pose before teleoperating. At zero, joints 2 and 3
  sit on their limits and Servo will refuse or crawl. Plan to a bent pose in RViz first.
* Planning in RViz and Servo both use `arm_controller`. Don't do both at once.
* `robot_supervisor` needs Jazzy or newer: it switches Servo's command type with
  `switch_command_type`, which Humble's Servo doesn't have.
