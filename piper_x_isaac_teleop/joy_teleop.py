"""Joystick -> RobotSupervisor (supervisor.py), arm and gripper.

The gripper axis sends the opening width (m) on /supervisor/gripper (Float64),
only when it changes.

The sticks send Cartesian velocity: Twist on /supervisor/velocity_target (unitless
stick values, base_link axes). Releasing the sticks sends a zero Twist, which stops
the robot.

Predefined poses (config/poses.yaml) are joint moves, triggered by their joystick
button or `ros2 topic pub --once /joy_teleop/goto std_msgs/msg/String "{data: home}"`:
    joints: [...] -> JointState on /supervisor/joint_command (move_group plans a
                     collision-free path)
    pose:   [...] -> Pose on /supervisor/joint_target (move_group plans a
                     collision-free path)
While a joint move runs the sticks are ignored; Start aborts it.

Default mapping (Xbox pad on Linux, check yours with `ros2 topic echo /joy`):
    left stick up/down     -> X
    left stick left/right  -> Y
    right stick up/down    -> Z
    hold Y (attitude)      -> sticks rotate about Y / X / Z instead
    A / B                  -> close / open gripper
    Start                  -> abort a joint move
    Back / X               -> home / tool_grab (poses.<name>.button)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64, String
from piper_x_isaac_teleop.utils import q_from_rpy, q_normalize


class JoyTeleop(Node):
    def __init__(self):
        # named poses come as `poses.<name>.*` overrides from config/poses.yaml
        super().__init__("joy_teleop", automatically_declare_parameters_from_overrides=True)

        def get_param(name, default):
            if self.has_parameter(name):
                return self.get_parameter(name).value
            return self.declare_parameter(name, default).value

        self.deadzone = get_param("deadzone", 0.1)
        self.ax = {
            'x': get_param('x', 1),
            'y': get_param('y', 0),
            'z': get_param('z', 2),
            'g': get_param('g', 3)
        }
        self.btn = {
            'attitude': get_param('attitude', 3),
        }
        self.arm_joints = list(get_param("arm_joints", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]))
        self.poses = self.load_poses()

        self.gripper_lim = get_param("gripper_lim", 0.1)
        self.last_grip = None

        self.vel_pub = self.create_publisher(Twist, "/supervisor/velocity_target", 10)
        self.joint_pub = self.create_publisher(Pose, "/supervisor/joint_target", 10)
        self.joint_cmd_pub = self.create_publisher(JointState, "/supervisor/joint_command", 10)
        self.grip_pub = self.create_publisher(Float64, "/supervisor/gripper", 10)
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.create_subscription(String, "/supervisor/state", self.on_supervisor_state, 10)

        self.prev_buttons = []
        self.sticks_active = False  # sticks were outside the deadzone last time
        self.joint_move = False     # we started a joint move and it hasn't ended yet

    def load_poses(self):
        """poses.<name>.joints: [6 x rad]  or  poses.<name>.pose: [x y z r p y] /
        [x y z qx qy qz qw] in planning_frame; optional poses.<name>.button."""
        poses = {}
        for key, param in self.get_parameters_by_prefix("poses").items():
            name, _, field = key.rpartition(".")
            poses.setdefault(name, {})[field] = param.value
        for name, p in list(poses.items()):
            if "joints" in p and len(p["joints"]) != len(self.arm_joints):
                self.get_logger().error(f"pose '{name}': joints needs {len(self.arm_joints)} values")
                del poses[name]
            elif "pose" in p and len(p["pose"]) not in (6, 7):
                self.get_logger().error(f"pose '{name}': pose needs [x y z r p y] or "
                                        "[x y z qx qy qz qw]")
                del poses[name]
            elif "joints" not in p and "pose" not in p:
                self.get_logger().error(f"pose '{name}': needs `joints` or `pose`")
                del poses[name]
        if poses:
            self.get_logger().info(
                "named poses: " + ", ".join(
                    f"{n}" + (f" (button {p['button']})" if "button" in p else "")
                    for n, p in poses.items()))
        return poses

    # --- inputs -----------------------------------------------------------------
    def axis(self, joy, key):
        i = self.ax[key]
        v = joy.axes[i] if i < len(joy.axes) else 0.0
        return v if abs(v) > self.deadzone else 0.0

    def pressed(self, joy, key):
        i = self.btn[key] if isinstance(key, str) else key
        return 0 <= i < len(joy.buttons) and joy.buttons[i] == 1

    def on_supervisor_state(self, msg: String):
        if msg.data in ('done', 'failed') and self.joint_move:
            self.joint_move = False
            self.get_logger().info(f"joint move {msg.data}")

    def on_joy(self, joy: Joy):
        # act on the rising edge of buttons only
        prev = self.prev_buttons
        def edge(k):
            i = self.btn[k] if isinstance(k, str) else k
            return self.pressed(joy, i) and not (0 <= i < len(prev) and prev[i] == 1)
        self.prev_buttons = list(joy.buttons)

        for name, p in self.poses.items():
            if "button" in p and edge(int(p["button"])):
                self.goto(name)

        if self.joint_move:  # sticks are not used during a joint move
            return
        x, y, z, g = self.axis(joy, 'x'), self.axis(joy, 'y'), self.axis(joy, 'z'), self.axis(joy, 'g')
        attitude = self.pressed(joy, 'attitude')
        active = bool(x or y or z)
        if active:
            self.send_velocity(x, y, z, attitude)
        elif self.sticks_active:  # just released
            self.stop()
        self.send_gripper(g)
        self.sticks_active = active

    # --- joystick -> supervisor ---------------------------------------------------
    def stop(self):
        self.vel_pub.publish(Twist())

    def send_velocity(self, x, y, z, attitude):
        t = Twist()
        if attitude:
            t.angular.x, t.angular.y, t.angular.z = y, x, z
        else:
            t.linear.x, t.linear.y, t.linear.z = x, y, z
        self.vel_pub.publish(t)

    def goto(self, name):
        """Joint move to a predefined pose."""
        if name not in self.poses:
            self.get_logger().warn(f"unknown pose '{name}', known: {list(self.poses)}")
            return

        p = self.poses[name]
        if "joints" in p:
            js = JointState(name=list(self.arm_joints),
                            position=[float(v) for v in p["joints"]])
            js.header.stamp = self.get_clock().now().to_msg()
            self.joint_cmd_pub.publish(js)
        else:
            v = [float(c) for c in p["pose"]]
            if len(v) == 6:
                # use euler
                quat = q_from_rpy(*v[3:])
            else:
                # use quaternion
                quat = q_normalize(v[3:])

            self.joint_pub.publish(self.to_pose(v[:3], quat))
        self.joint_move = True
        self.get_logger().info(f"joint move to {name}")

    @staticmethod
    def to_pose(pos, quat):
        p = Pose()
        p.position.x, p.position.y, p.position.z = pos
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        return p

    # --- gripper ------------------------------------------------------------------
    def send_gripper(self, val):
        width = float(val * self.gripper_lim)
        # Joy publishes at ~50 Hz; only send when the target actually changes.
        if self.last_grip is not None and abs(width - self.last_grip) < 1e-2:
            return
        self.last_grip = width
        self.grip_pub.publish(Float64(data=width))


def main():
    rclpy.init()
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
