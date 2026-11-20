import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, TwistStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, JointConstraint, MoveItErrorCodes,
                             OrientationConstraint, PositionConstraint, ServoStatus)
from moveit_msgs.srv import GetCartesianPath, ServoCommandType
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64, Header, String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from piper_x_isaac_teleop.utils import q_conj, q_mul, q_normalize, q_to_rotvec

ERROR_NAMES = {v: k for k, v in vars(MoveItErrorCodes).items()
               if k.isupper() and isinstance(v, int)}


class RobotSupervisor(Node):
    """ wrapper for MoveIt overcomplicated interface.

        subscriptions:
            /supervisor/linear_target (Pose) - moves to given pose by linear movement
            /supervisor/joint_target (Pose) - moves to given pose by joint movement,
                planned by move_group (collision-free path)
            /supervisor/velocity_target (Twist) - moves by given linear velocity
            /supervisor/joint_command (JointState) - moves to given joint positions,
                planned by move_group (collision-free path)
            /supervisor/gripper (Float64) - gripper opening width (m); sent to
                gripper_controller as a one-point trajectory lasting
                |change| / gripper_speed. Independent of the arm tasks.
        publishes:
            /supervisor/state (String) - one of 'done', 'busy', 'failed'

        Poses are ee_frame poses in planning_frame. Twists are in command_frame,
        in Servo's command_in_type units (unitless +-1 by default, scaled by Servo's
        scale.linear / scale.rotational). Velocity motion lasts while new Twists
        arrive (velocity_timeout). A zero Twist stops any motion (a stopped
        linear / planned move ends as 'failed').

        A new target replaces the running one:
            linear   -> Servo pose tracking, straight-line EE path; done when the
                        EE is within linear_tolerance / angular_tolerance. The
                        segment is checked collision-free first (move_group
                        /compute_cartesian_path); a blocked path or a Servo halt
                        for collision ends the move as 'failed'
            velocity -> Servo twist commands
            planned  -> joint_target / joint_command: MoveGroup action (/move_action)
                        with pose (linear_tolerance / angular_tolerance) or joint
                        (joint_tolerance) goal constraints. move_group solves IK,
                        plans a collision-free path and executes it on the arm
                        controller; done / failed from the action result
        A move that does not finish within move_timeout ends as 'failed'.
    """

    def __init__(self):
        super().__init__("robot_supervisor")
        get_param = lambda name, default: self.declare_parameter(name, default).value
        self.command_frame = get_param("command_frame", "base_link")
        self.planning_frame = get_param("planning_frame", "base_link")
        self.ee_frame = get_param("ee_frame", "tcp_link")
        self.move_group = get_param("move_group", "arm")
        self.arm_joints = list(get_param(
            "arm_joints", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]))
        self.linear_tolerance = get_param("linear_tolerance", 0.002)     # m
        self.angular_tolerance = get_param("angular_tolerance", 0.01)    # rad
        self.joint_tolerance = get_param("joint_tolerance", 0.005)       # rad
        self.move_timeout = get_param("move_timeout", 30.0)              # s
        self.path_check_step = get_param("path_check_step", 0.005)       # m, linear check
        self.velocity_timeout = get_param("velocity_timeout", 0.2)       # s
        # planned (MoveGroup) moves
        self.planning_time = get_param("planning_time", 5.0)             # s
        self.planning_attempts = get_param("planning_attempts", 3)
        self.velocity_scaling = get_param("velocity_scaling", 0.2)       # 0..1
        self.acceleration_scaling = get_param("acceleration_scaling", 0.2)
        # gripper
        self.gripper_joint = get_param("gripper_joint", "gripper")
        gripper_ctrl = get_param("gripper_controller", "gripper_controller")
        self.gripper_speed = get_param("gripper_speed", 0.5)             # m/s of width
        self.gripper_lim = get_param("gripper_lim", 0.1)                 # m, full stroke
        self.gripper_width = None   # last commanded width

        self.state = 'done'
        self.task = None            # None, 'linear', 'planned' or 'velocity'
        self.plan_goal = None       # planned: MoveGroup goal handle
        self.task_id = 0            # identifies the running task in async callbacks
        self.deadline = None        # linear / planned: give up after this (s)
        self.goal_pose = None       # linear: ((x, y, z), quat)
        self.path_ok = False        # linear: straight path checked collision-free
        self.servo_since = None     # linear: when commands to Servo started (s)
        self.twist = None           # velocity: latest Twist and when it arrived
        self.twist_time = None
        self.mode = None            # Servo command type, known once Servo accepted it
        self.switching = False

        self.state_pub = self.create_publisher(String, "/supervisor/state", 10)
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/servo_node/pose_target_cmds", 10)
        self.grip_pub = self.create_publisher(
            JointTrajectory, f"/{gripper_ctrl}/joint_trajectory", 10)

        self.create_subscription(Pose, "/supervisor/linear_target", self.on_linear_target, 10)
        self.create_subscription(Pose, "/supervisor/joint_target", self.on_joint_target, 10)
        self.create_subscription(Twist, "/supervisor/velocity_target", self.on_velocity_target, 10)
        self.create_subscription(JointState, "/supervisor/joint_command", self.on_joint_command, 10)
        self.create_subscription(Float64, "/supervisor/gripper", self.on_gripper, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.mode_cli = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        self.cartesian_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.create_subscription(ServoStatus, "/servo_node/status", self.on_servo_status, 10)
        self.move_action = ActionClient(self, MoveGroup, "/move_action")
        self.create_timer(0.02, self.step)
        self.publish_state()

    # --- state ------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_state(self):
        self.state_pub.publish(String(data=self.state))

    def start(self, task, description):
        """Replace whatever runs with a new task."""
        if self.task is not None and self.task != task:
            self.get_logger().info(f"{self.task} move replaced by {task} move")
        self.cancel_plan()
        self.task = task
        self.task_id += 1
        self.deadline = self.now() + self.move_timeout
        self.goal_pose = None
        self.path_ok = False
        self.servo_since = None
        if self.state != 'busy':
            self.state = 'busy'
            self.publish_state()
        if description:
            self.get_logger().info(f"{task} move to {description}")

    def finish(self, result, reason=""):
        """End the running task with 'done' or 'failed'."""
        if result == 'failed':
            self.get_logger().warn(f"{self.task} move failed: {reason}")
            self.cancel_plan()
        elif self.task != 'velocity':
            self.get_logger().info(f"{self.task} move done")
        self.plan_goal = None
        self.task = None
        self.goal_pose = self.twist = None
        self.state = result
        self.publish_state()

    # --- servo mode -------------------------------------------------------------
    def ensure_mode(self, mode):
        """True if Servo is in `mode`, else start switching to it and return False.
        Our mode changes only once Servo accepts."""
        if self.mode == mode:
            return True
        if self.switching:
            return False
        if not self.mode_cli.service_is_ready():
            self.get_logger().info("waiting for servo_node ...", throttle_duration_sec=5.0)
            return False
        req = ServoCommandType.Request()
        req.command_type = {"twist": ServoCommandType.Request.TWIST,
                            "pose": ServoCommandType.Request.POSE}[mode]
        self.switching = True

        def done(future):
            self.switching = False
            res = future.result()
            if res is not None and res.success:
                self.mode = mode
                self.get_logger().info(f"Servo mode: {mode}")
            else:
                self.get_logger().warn(f"Servo refused switching to {mode} mode",
                                       throttle_duration_sec=2.0)

        self.mode_cli.call_async(req).add_done_callback(done)
        return False

    def ee_pose(self):
        """Current EE pose in the planning frame, or None if TF is not available."""
        try:
            tf = self.tf_buffer.lookup_transform(self.planning_frame, self.ee_frame, Time())
        except Exception as e:  # noqa: BLE001 - tf2 raises several exception types
            self.get_logger().warn(f"no TF {self.planning_frame} -> {self.ee_frame}: {e}",
                                   throttle_duration_sec=2.0)
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)

    # --- inputs -----------------------------------------------------------------
    def on_linear_target(self, msg: Pose):
        p, o = msg.position, msg.orientation
        self.move_pose_linear(((p.x, p.y, p.z), q_normalize((o.x, o.y, o.z, o.w))))

    def on_joint_target(self, msg: Pose):
        p, o = msg.position, msg.orientation
        self.move_pose_planned(((p.x, p.y, p.z), q_normalize((o.x, o.y, o.z, o.w))))

    def on_joint_command(self, msg: JointState):
        goal = dict(zip(msg.name, msg.position))
        missing = [j for j in self.arm_joints if j not in goal]
        if missing:
            self.start('planned', "")
            self.finish('failed', f"joint command misses {missing}")
            return
        self.move_joints_planned({j: float(goal[j]) for j in self.arm_joints})

    def on_velocity_target(self, msg: Twist):
        self.move_vel_linear(msg)

    # --- gripper: one-point trajectory on gripper_controller ------------------------
    def on_gripper(self, msg: Float64):
        width = float(msg.data)
        dist = abs(width - self.gripper_width) if self.gripper_width is not None \
            else self.gripper_lim
        t = max(dist / self.gripper_speed, 0.02)
        self.gripper_width = width
        traj = JointTrajectory(joint_names=[self.gripper_joint])
        traj.points = [JointTrajectoryPoint(
            positions=[width],
            time_from_start=Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9)))]
        self.grip_pub.publish(traj)

    # --- velocity: Servo twist ----------------------------------------------------
    def move_vel_linear(self, twist: Twist):
        """Cartesian velocity in command_frame; zero stops."""
        v = (twist.linear.x, twist.linear.y, twist.linear.z,
             twist.angular.x, twist.angular.y, twist.angular.z)
        if not any(v):  # stop whatever runs
            if self.task == 'velocity':
                self.finish('done')
            elif self.task is not None:
                self.finish('failed', "stopped by zero velocity command")
            return
        if self.task != 'velocity':
            self.start('velocity', "")
        self.twist, self.twist_time = twist, self.now()

    def step_velocity(self):
        if self.now() - self.twist_time > self.velocity_timeout:
            self.finish('done')  # no new command: stop
            return
        if not self.ensure_mode("twist"):
            return
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.command_frame
        t.twist = self.twist
        self.twist_pub.publish(t)

    # --- linear: Servo pose tracking ----------------------------------------------
    def move_pose_linear(self, pose):
        """Move the EE to `pose` ((x, y, z), quat) along a straight line.

        Obstacles (planning scene: collision objects, octomap) can't be avoided on a
        straight line, so the move is refused or stopped instead:
          * a new move first checks the whole segment with move_group's
            /compute_cartesian_path (avoid_collisions) and fails if it is blocked
          * while moving, Servo's collision checking slows down and halts near
            obstacles; a halt ends the move as 'failed'
        A stream of targets (e.g. from a joystick) updates the running move; the
        updates are short steps guarded by Servo only."""
        if self.task == 'linear':
            self.deadline = self.now() + self.move_timeout
            self.goal_pose = pose
            return
        self.start('linear', f"{tuple(round(c, 3) for c in pose[0])}")
        self.goal_pose = pose
        self.check_linear_path(pose)

    def check_linear_path(self, pose):
        if not self.cartesian_cli.service_is_ready():
            self.finish('failed', "/compute_cartesian_path not available "
                                  "(is move_group running?), can't check obstacles")
            return
        req = GetCartesianPath.Request()
        req.header.frame_id = self.planning_frame
        req.start_state.is_diff = True  # from the current state
        req.group_name = self.move_group
        req.link_name = self.ee_frame
        (x, y, z), (qx, qy, qz, qw) = pose
        req.waypoints = [Pose(position=Point(x=x, y=y, z=z),
                              orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))]
        req.max_step = self.path_check_step
        req.jump_threshold = 0.0    # disabled
        req.avoid_collisions = True
        task_id = self.task_id
        self.cartesian_cli.call_async(req).add_done_callback(
            lambda f: self.on_linear_path(f, task_id))

    def on_linear_path(self, future, task_id):
        if task_id != self.task_id:
            return  # replaced meanwhile
        res = future.result()
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            code = res.error_code.val if res is not None else "no response"
            self.finish('failed', f"straight path check failed ({ERROR_NAMES.get(code, code)})")
        elif res.fraction < 0.999:
            self.finish('failed', f"straight path blocked after {res.fraction * 100:.0f} % "
                                  "(collision or unreachable)")
        else:
            self.path_ok = True

    def on_servo_status(self, msg: ServoStatus):
        if msg.code != ServoStatus.HALT_FOR_COLLISION:
            return
        if self.task == 'linear' and self.servo_since is not None \
                and self.now() - self.servo_since > 0.2:  # ignore a status from before
            self.finish('failed', f"Servo halted for collision: {msg.message}")
        elif self.task == 'velocity':
            self.get_logger().warn("Servo halted for collision", throttle_duration_sec=2.0)

    def step_linear(self):
        if not self.path_ok or not self.ensure_mode("pose"):
            return  # waiting for the path check or Servo's mode switch
        if self.servo_since is None:
            self.servo_since = self.now()
        pos, quat = self.goal_pose
        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = self.planning_frame
        p.pose.position.x, p.pose.position.y, p.pose.position.z = pos
        (p.pose.orientation.x, p.pose.orientation.y,
         p.pose.orientation.z, p.pose.orientation.w) = quat
        self.pose_pub.publish(p)

        cur = self.ee_pose()
        if cur is None:
            return
        dist = math.dist(pos, cur[0])
        ang = math.sqrt(sum(c * c for c in q_to_rotvec(q_mul(quat, q_conj(cur[1])))))
        if dist < self.linear_tolerance and ang < self.angular_tolerance:
            self.finish('done')

    # --- planned: MoveGroup action (joint values or EE pose) -----------------------
    def move_joints_planned(self, goal):
        """Plan (collision-free) and execute a move to {joint: position}."""
        self.start('planned', ", ".join(f"{j}={v:.3f}" for j, v in goal.items()))
        self.send_plan(Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=v,
                            tolerance_above=self.joint_tolerance,
                            tolerance_below=self.joint_tolerance, weight=1.0)
            for j, v in goal.items()]))

    def move_pose_planned(self, pose):
        """Plan (collision-free) and execute a move of ee_frame to `pose`
        ((x, y, z), quat) in planning_frame. move_group solves the IK."""
        self.start('planned', f"{tuple(round(c, 3) for c in pose[0])}")
        (x, y, z), (qx, qy, qz, qw) = pose
        region = BoundingVolume(
            primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE,
                                       dimensions=[self.linear_tolerance])],
            primitive_poses=[Pose(position=Point(x=x, y=y, z=z),
                                  orientation=Quaternion(w=1.0))])
        header = Header(frame_id=self.planning_frame)
        self.send_plan(Constraints(
            position_constraints=[PositionConstraint(
                header=header, link_name=self.ee_frame,
                constraint_region=region, weight=1.0)],
            orientation_constraints=[OrientationConstraint(
                header=header, link_name=self.ee_frame,
                orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
                absolute_x_axis_tolerance=self.angular_tolerance,
                absolute_y_axis_tolerance=self.angular_tolerance,
                absolute_z_axis_tolerance=self.angular_tolerance,
                weight=1.0)]))

    def send_plan(self, goal_constraints):
        """MoveGroup goal from the current state; move_group plans a collision-free
        path and executes it on the arm controller (Servo is not used)."""
        if not self.move_action.server_is_ready():
            self.finish('failed', "/move_action not available (is move_group running?)")
            return
        g = MoveGroup.Goal()
        req = g.request
        req.group_name = self.move_group
        req.num_planning_attempts = self.planning_attempts
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.velocity_scaling
        req.max_acceleration_scaling_factor = self.acceleration_scaling
        req.start_state.is_diff = True  # plan from the current state
        req.goal_constraints = [goal_constraints]
        g.planning_options.plan_only = False
        g.planning_options.replan = False
        task_id = self.task_id
        self.move_action.send_goal_async(g).add_done_callback(
            lambda f: self.on_plan_accepted(f, task_id))

    def on_plan_accepted(self, future, task_id):
        handle = future.result()
        if task_id != self.task_id:  # replaced meanwhile
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        if handle is None or not handle.accepted:
            self.finish('failed', "move_group rejected the goal")
            return
        self.plan_goal = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_plan_result(f, task_id))

    def on_plan_result(self, future, task_id):
        if task_id != self.task_id:
            return
        res = future.result()
        code = res.result.error_code.val if res is not None else None
        self.plan_goal = None
        if code == MoveItErrorCodes.SUCCESS:
            self.finish('done')
        else:
            self.finish('failed', f"move_group error {ERROR_NAMES.get(code, code)}")

    def cancel_plan(self):
        if self.plan_goal is not None:
            self.plan_goal.cancel_goal_async()
            self.plan_goal = None

    # --- timer --------------------------------------------------------------------
    def step(self):
        if self.task is None:
            return
        if self.task != 'velocity' and self.now() > self.deadline:
            self.finish('failed', f"not reached in {self.move_timeout:.0f} s")
            return
        {'velocity': self.step_velocity,
         'linear': self.step_linear,
         'planned': lambda: None}[self.task]()  # planned: move_group reports back


def main():
    rclpy.init()
    node = RobotSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
