"""Translate between the ROS-side gripper model and the Isaac Sim asset.

ROS side (MoveIt / ros2_control) drives one virtual joint "gripper" = opening
width (0 ... 0.1 m), with the fingers as mimic joints. The Isaac asset has no
virtual joint, only the two real fingers:
    gripper_joint1 =  width / 2   (0 ... 0.05)
    gripper_joint2 = -width / 2   (-0.05 ... 0)

    /ros_joint_commands  --(expand gripper)-->   /isaac_joint_commands
    /isaac_joint_states  --(add gripper)----->   /ros_joint_states
All other joints are passed through unchanged.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class IsaacBridge(Node):
    def __init__(self):
        super().__init__("isaac_bridge")
        self.gripper = self.declare_parameter("gripper_joint", "gripper").value
        self.f1, self.f2 = self.declare_parameter(
            "finger_joints", ["gripper_joint1", "gripper_joint2"]).value

        self.to_isaac = self.create_publisher(JointState, "/isaac_joint_commands", 10)
        self.to_ros = self.create_publisher(JointState, "/ros_joint_states", 10)
        self.create_subscription(JointState, "/ros_joint_commands", self.on_command, 10)
        self.create_subscription(JointState, "/isaac_joint_states", self.on_state, 10)

    def on_command(self, msg: JointState):
        names, positions = [], []
        for name, pos in zip(msg.name, msg.position):
            if name == self.gripper:
                names += [self.f1, self.f2]
                positions += [pos / 2.0, -pos / 2.0]
            else:
                names.append(name)
                positions.append(pos)
        self.to_isaac.publish(JointState(header=msg.header, name=names, position=positions))

    def on_state(self, msg: JointState):
        names, positions = list(msg.name), list(msg.position)
        velocities = list(msg.velocity) if len(msg.velocity) == len(msg.name) else []
        idx = {n: i for i, n in enumerate(names)}
        if self.f1 in idx and self.f2 in idx:
            a, b = idx[self.f1], idx[self.f2]
            names.append(self.gripper)
            positions.append(positions[a] - positions[b])
            if velocities:
                velocities.append(velocities[a] - velocities[b])
        self.to_ros.publish(JointState(header=msg.header, name=names,
                                       position=positions, velocity=velocities))


def main():
    rclpy.init()
    node = IsaacBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
