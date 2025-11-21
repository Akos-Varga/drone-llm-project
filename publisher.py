#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from anafi_autonomy.msg import PoseCommand
from geometry_msgs.msg import Pose
import math
import time

class PosePublisher(Node):
    """Publishes PoseCommand messages and reads the current Pose from /anafi/drone/pose."""

    def __init__(self, drone_name: str, name_map: dict):
        self.drone_name = drone_name
        if self.drone_name not in name_map:
            raise ValueError(f"No mapping found for {drone_name}.")
        self.anafi_name = name_map[self.drone_name]
        super().__init__(f"{self.drone_name}_pose_publisher")
    
        # --- Publisher (PoseCommand) ---
        self.cmd_topic = f"{self.anafi_name}/drone/reference/pose"
        self.publisher = self.create_publisher(PoseCommand, self.cmd_topic, 10)
        self.get_logger().info(f"Publishing PoseCommand → {self.cmd_topic}")

        # --- Subscriber (Pose) ---
        self.pose_topic = f"{self.anafi_name}/drone/pose"
        self.current_pose = None
        self.create_subscription(Pose, self.pose_topic, self._pose_callback, 10)
        self.get_logger().info(f"Subscribed to drone pose → {self.pose_topic}")

        # --- Arrival tolerances ---
        self.pos_tolerance = 0.1
        self.yaw_tolerance = 5.0


    def send_pose(self, pos, yaw_deg, frame_id = "map"):
        """Send a single PoseCommand message and wait until drone arrives"""
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
            f"Sent PoseCommand: frame='{frame_id}', "
            f"x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}, yaw={msg.yaw:.1f}°"
        )

        # Wait for drone to arrive
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
                self.get_logger().info(f"Arrived at goal (dist={dist:.2f}, yaw_err={yaw_err:.1f})")


    def _pose_callback(self, msg: Pose):
        """Callback to store the latest /anafi/drone/pose message."""
        self.current_pose = msg

    def get_pose(self):
        """Return the most recently received drone pose (or None if nothing received yet)."""
        return self.current_pose
    
    def move_and_execute(self, goal, altitude, t, obj, skill):
        """Move to an object at a certain altitude and execute task."""
        if self.current_pose is None:
            self.get_logger().warn("No current pose available yet.")
            return
        
        # Move drone to flying altitude
        self.send_pose((self.current_pose.x, self.current_pose.y, altitude), self.current_pose.yaw)
        
        # Compute yaw toward the goal
        dx = goal[0] - self.current_pose.x
        dy = goal[1] - self.current_pose.y
        yaw_rad = math.atan2(dy, dx)
        yaw_deg = math.degrees(yaw_rad)
        self.get_logger().info(f"Moving toward ({goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f})")

        # Send drone to goal pos
        self.send_pose((goal[0], goal[1], altitude), yaw_deg)

        # Adjust z
        self.send_pose((self.current_pose.x, self.current_pose.y, goal[2]), yaw_deg)
        
        # Execute skill if provided
        self.get_logger().info(f"Executing skill '{skill}' on '{obj}'")
        start = time.time()
        while rclpy.ok() and (time.time() - start < t):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"Finished waiting at target after {t:.1f}s.")


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher("Drone1")

    node.get_logger().info("Waiting for first /anafi1/drone/pose message...")
    while node.get_pose() is None and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info("Got first pose from drone!")

    while node.publisher.get_subscription_count() == 0 and rclpy.ok():
        node.get_logger().info("Waiting for drone subscriber...")
        rclpy.spin_once(node, timeout_sec=0.1)

    node.move_and_execute(goal=(1.0, 0.0, 2.0), altitude=1, t=3, obj="Tower1", skill="CaptureRGBImage")
    node.move_and_execute(goal=(2.5, -1.0, 3.5), altitude=2, t=5, obj="RoofTop1", skill="InspectStructure")
    
    node.get_logger().info("Mission complete.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()