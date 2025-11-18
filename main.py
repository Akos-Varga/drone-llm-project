import threading
import rclpy
from publisher import PosePublisher
from world import skills, objects, drones
from main_pipeline import pipeline
from test_tasks import task_list

def wait_for_first_pose(node: PosePublisher):
    node.get_logger().info(f"[{node.drone_prefix}] Waiting for first pose...")
    while rclpy.ok() and node.get_pose() is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info(f"[{node.drone_prefix}] Got first pose!")

def wait_for_subscriber(node: PosePublisher):
    while rclpy.ok() and node.publisher.get_subscription_count() == 0:
        node.get_logger().info(f"[{node.drone_prefix}] Waiting for cmd subscriber...")
        rclpy.spin_once(node, timeout_sec=0.1)

def fly_mission(node: PosePublisher, drone_name, altitude, drone_schedule, skills, objects):
    wait_for_first_pose(node)
    wait_for_subscriber(node)
    for subtask in drone_schedule:
        # print(f"Goal: {objects[subtask["object"]]}\nEx. time: {skills[subtask["skill"]]}\nObject: {subtask["object"]}\nSkill: {subtask["skill"]}\n", "="*90)
        node.move_and_execute(goal=objects[subtask["object"]], altitude=altitude, t=skills[subtask["skill"]], obj=subtask["object"], skill=subtask["skill"])
    node.get_logger().info(f"[{drone_name}]: Mission complete.")
    # print(f"{drone_name}: mission complete.")


if __name__ == "__main__":
    rclpy.init()

    # Create one node per drone and get its starting pose
    pose_publishers = {}
    flight_altitudes = {}

    try:
        for i, drone_name in enumerate(drones.keys()):
            # Create nodes for drones
            node = PosePublisher()
            pose_publishers[drone_name] = node
            flight_altitudes[drone_name] = i + 1

            # Get start pos
            wait_for_first_pose(node)
            start_pose = node.get_pose()
            drones[drone_name]["pos"] = start_pose
            node.get_logger().info(f"[{drone_name}] Start pose: {start_pose}")

        # Plan mission
        task = task_list[0]["task"]
        results = pipeline("gpt-5-mini", task, skills, objects, drones)
        schedule = results["schedule"]

        threads = []

        # Create a thread for each drone that has tasks
        for drone_name, drone_schedule in schedule.items():
            if not drone_schedule:
                continue  # skip drones with no tasks

            node = pose_publishers.get(drone_name)

            t = threading.Thread(
                target=fly_mission,
                args=(node, drone_name, flight_altitudes["drone_name"], drone_schedule, skills, objects),
                daemon=True,
            )
            threads.append(t)
            t.start()

        # Wait for all mission threads to finish
        for t in threads:
            t.join()

    except KeyboardInterrupt:
        pass
    finally:
        # Destroy nodes and shutdown
        for n in list(rclpy.get_default_context().get_nodes()):
            try:
                n.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()
