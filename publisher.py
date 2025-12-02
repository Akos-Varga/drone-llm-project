#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from anafi_autonomy.msg import PoseCommand
from anafi_autonomy.msg import KeyboardCommand

import math
import time


class SimplePose:
    def __init__(self, d):
        self.x = d["x"]
        self.y = d["y"]
        self.z = d["z"]
        self.qx = d["qx"]
        self.qy = d["qy"]
        self.qz = d["qz"]
        self.qw = d["qw"]

    @property
    def yaw(self):
        """Compute yaw from quaternion."""
        # yaw = atan2(2(wz + xy), 1 - 2(y² + z²))
        siny_cosp = 2.0 * (self.qw * self.qz + self.qx * self.qy)
        cosy_cosp = 1.0 - 2.0 * (self.qy * self.qy + self.qz * self.qz)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))


class PosePublisher(Node):
    """Publishes PoseCommand and reads PoseStamped from /vicon/parrot_4K/parrot_4K."""

    def __init__(self, drone_name: str, name_map: dict):
        self.drone_name = drone_name

        if self.drone_name not in name_map:
            raise ValueError(f"No mapping found for {drone_name}.")

        base_ns = name_map[self.drone_name]
        self.base_ns = base_ns
        # self.anafi_node_name = f"{base_ns}/anafi"

        super().__init__(f"{self.drone_name}_pose_publisher")

        # ---------------- Publisher ----------------
        self.cmd_topic = f"{self.base_ns}/drone/reference/pose"
        self.publisher = self.create_publisher(PoseCommand, self.cmd_topic, 10)
        self.get_logger().info(f"Publishing PoseCommand → {self.cmd_topic}")

        # ---------------- Subscriber ----------------
        self.pose_topic = f"{self.base_ns}/drone/pose"
        self.current_pose = None

        self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self._pose_callback,
            10
        )
        self.get_logger().info(f"Subscribed to PoseStamped → {self.pose_topic}")

        # ---------------- Keyboard publisher ----------------
        self.keyboard_pub = self.create_publisher(
            KeyboardCommand,
            f"{self.base_ns}/keyboard/command",
            10
        )   
        self.get_logger().info(f"Publishing keyboard commands → {self.base_ns}/keyboard/command")

        # ---------------- Arrival tolerances ----------------
        self.pos_tolerance = 0.1
        self.yaw_tolerance = 5.0

    # ---------------- Pose callback ----------------

    def _pose_callback(self, msg: PoseStamped):
        self.current_pose = {
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
            "qx": msg.pose.orientation.x,
            "qy": msg.pose.orientation.y,
            "qz": msg.pose.orientation.z,
            "qw": msg.pose.orientation.w,
        }

        # self.get_logger().info(
        #     f"Pose → x:{msg.pose.position.x:.3f} "
        #     f"y:{msg.pose.position.y:.3f} "
        #     f"z:{msg.pose.position.z:.3f}"
        # )

    # ---------------- Pose getter ----------------

    def get_pose(self):
        if self.current_pose is None:
            return None
        return SimplePose(self.current_pose)

    # ---------------- Sending pose commands ----------------

    def send_pose(self, pos, yaw_deg, frame_id="map"):
        msg = PoseCommand()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.x = float(pos[0])
        msg.y = float(pos[1])
        msg.z = float(pos[2])
        msg.yaw = float(yaw_deg)

        self.publisher.publish(msg)

        self.get_logger().info(
            f"Sent PoseCommand: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}, yaw={msg.yaw:.1f}°"
        )

        # Wait until the drone arrives
        arrived = False
        while rclpy.ok() and not arrived:
            rclpy.spin_once(self, timeout_sec=0.1)
            current = self.get_pose()
            if current is None:
                continue

            dx = pos[0] - current.x
            dy = pos[1] - current.y
            dz = pos[2] - current.z
            dist = math.sqrt(dx**2 + dy**2 + dz**2)

            yaw_err = abs((yaw_deg - current.yaw + 180) % 360 - 180)

            if dist <= self.pos_tolerance and yaw_err <= self.yaw_tolerance:
                arrived = True
                self.get_logger().info(
                    f"Arrived (dist={dist:.2f}, yaw_err={yaw_err:.1f})"
                )

    # ---------------- Arm, Takeoff, Land and Offboard ----------------

    def arm(self):
        msg = KeyboardCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "body"
        msg.drone_action = 1  # arm
        self.keyboard_pub.publish(msg)
        self.get_logger().info("Sent ARM command (drone_action=1)")

    def disarm(self):
        msg = KeyboardCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "body"
        msg.drone_action = 5  # disarm
        self.keyboard_pub.publish(msg)
        self.get_logger().info("Sent DISARM command (drone_action=5)")
    
    def takeoff(self):
        msg = KeyboardCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "body"
        msg.drone_action = 2  # takeoff
        self.keyboard_pub.publish(msg)
        self.get_logger().info("Sent TAKEOFF command (drone_action=2)")

    def land(self):
        msg = KeyboardCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "body"
        msg.drone_action = 4  # land
        self.keyboard_pub.publish(msg)
        self.get_logger().info("Sent LAND command (drone_action=4)")

    def offboard(self):
        msg = KeyboardCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "body"
        msg.drone_action = 102  # offboard mode
        self.keyboard_pub.publish(msg)
        self.get_logger().info("Sent OFFBOARD command (drone_action=102)")

    # ---------------- High-level task ----------------

    def move_and_execute(self, goal, altitude, t, obj, skill):
        if self.current_pose is None:
            self.get_logger().warn("No current pose available yet.")
            return

        pose = self.get_pose()

        # Step 1: go to flight altitude
        self.send_pose((pose.x, pose.y, altitude), pose.yaw)

        # Step 2: yaw toward goal
        dx = goal[0] - pose.x
        dy = goal[1] - pose.y
        yaw_deg = math.degrees(math.atan2(dy, dx))

        self.get_logger().info(
            f"Turning toward ({goal[0]:.2f}, {goal[1]:.2f}) with yaw={yaw_deg:.1f}°"
        )

        # Step 3: move horizontally to target
        self.send_pose((goal[0], goal[1], altitude), yaw_deg)

        # Step 4: descend/ascend to goal Z
        pose = self.get_pose()
        self.send_pose((pose.x, pose.y, goal[2]), yaw_deg)

        # Step 5: execute skill
        self.get_logger().info(f"Executing skill '{skill}' on '{obj}'...")
        start = time.time()
        while rclpy.ok() and (time.time() - start < t):
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f"Task complete after {t:.1f}s.")


def main(args=None):
    DRONE_TO_NODE = {
    "Drone1": "anafi",
    #"Drone2": "anafi2",
    }
    rclpy.init(args=args)
    node = PosePublisher("Drone1", DRONE_TO_NODE)

    node.get_logger().info("Waiting for first pose message...")
    while node.get_pose() is None and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info("Got first pose from drone!")

    while node.publisher.get_subscription_count() == 0 and rclpy.ok():
        node.get_logger().info("Waiting for drone subscriber...")
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info("Drone subscriber connected!")

    #while rclpy.ok():
#
    #    node.send_pose((1,0,2), 20)
#
    #    rclpy.spin_once(node, timeout_sec=0.1)
    #    time.sleep(0.2)

    try:
        node.arm()
        time.sleep(1)
        node.takeoff()
        time.sleep(3)
        node.offboard()

        node.move_and_execute(goal=(1.0, 2.0, 1.0), altitude=1, t=3, obj="Tower1", skill="CaptureRGBImage")
        node.move_and_execute(goal=(2.5, 2.0, 1.5), altitude=2, t=5, obj="RoofTop1", skill="InspectStructure")
        
        node.get_logger().info("Mission complete.")
    finally:
        node.land()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
