"""Joystick -> MoveIt Servo twist commands + gripper open/close.

Default mapping (Xbox pad on Linux, check yours with `ros2 topic echo /joy`):
    left stick up/down     -> linear X
    left stick left/right  -> linear Y
    right stick up/down    -> linear Z
    right stick left/right -> yaw (angular Z)
    LB / RB                -> roll (angular X) -/+
    A / B                  -> open / close gripper
"""
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:  # Jazzy and newer
    from moveit_msgs.srv import ServoCommandType
except ImportError:  # Humble
    ServoCommandType = None


class JoyTeleop(Node):
    def __init__(self):
        super().__init__("joy_teleop")
        p = lambda name, default: self.declare_parameter(name, default).value
        self.frame = p("command_frame", "base_link")
        self.deadzone = p("deadzone", 0.1)
        self.ax = {k: p(f"axis_{k}", d) for k, d in
                   [("x", 1), ("y", 0), ("z", 4), ("yaw", 3)]}
        self.btn = {k: p(f"button_{k}", d) for k, d in
                    [("open", 0), ("close", 1), ("roll_neg", 4), ("roll_pos", 5)]}
        self.gripper_joint = p("gripper_joint", "gripper")
        self.gripper_open = p("gripper_open", 0.1)
        self.gripper_closed = p("gripper_closed", 0.0)
        gripper_ctrl = p("gripper_controller", "gripper_controller")

        self.twist_pub = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.grip_pub = self.create_publisher(
            JointTrajectory, f"/{gripper_ctrl}/joint_trajectory", 10)
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.prev_buttons = []

        # Start Servo once it is up (Humble: start_servo, Jazzy: twist mode)
        self.start_cli = self.create_client(Trigger, "/servo_node/start_servo")
        self.mode_cli = (self.create_client(ServoCommandType, "/servo_node/switch_command_type")
                         if ServoCommandType else None)
        self.start_timer = self.create_timer(1.0, self.start_servo)

    def start_servo(self):
        if self.mode_cli and self.mode_cli.service_is_ready():
            req = ServoCommandType.Request()
            req.command_type = ServoCommandType.Request.TWIST
            self.mode_cli.call_async(req)
        elif self.start_cli.service_is_ready():
            self.start_cli.call_async(Trigger.Request())
        else:
            self.get_logger().info("waiting for servo_node ...", throttle_duration_sec=5.0)
            return
        self.get_logger().info("Servo started in twist mode")
        self.start_timer.cancel()

    def axis(self, joy, key):
        i = self.ax[key]
        v = joy.axes[i] if i < len(joy.axes) else 0.0
        return v if abs(v) > self.deadzone else 0.0

    def pressed(self, joy, key):
        i = self.btn[key]
        return i < len(joy.buttons) and joy.buttons[i] == 1

    def on_joy(self, joy: Joy):
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame
        t.twist.linear.x = self.axis(joy, "x")
        t.twist.linear.y = self.axis(joy, "y")
        t.twist.linear.z = self.axis(joy, "z")
        t.twist.angular.z = self.axis(joy, "yaw")
        t.twist.angular.x = (float(self.pressed(joy, "roll_pos"))
                             - float(self.pressed(joy, "roll_neg")))
        self.twist_pub.publish(t)

        # gripper: act on the rising edge of A / B only
        prev = self.prev_buttons
        edge = lambda k: self.pressed(joy, k) and not (
            self.btn[k] < len(prev) and prev[self.btn[k]] == 1)
        if edge("open"):
            self.send_gripper(self.gripper_open)
        elif edge("close"):
            self.send_gripper(self.gripper_closed)
        self.prev_buttons = list(joy.buttons)

    def send_gripper(self, width):
        traj = JointTrajectory(joint_names=[self.gripper_joint])
        traj.points = [JointTrajectoryPoint(positions=[float(width)],
                                            time_from_start=Duration(sec=0, nanosec=500_000_000))]
        self.grip_pub.publish(traj)


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
