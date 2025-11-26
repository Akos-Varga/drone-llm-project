import threading
import rclpy
from publisher import PosePublisher
from model_discovery import AnafiModelScanner
from worlds.real_world import skills, objects, drones
from main_pipeline import pipeline

# DRONE_TO_NODE = {
#     "Drone1": "anafi1",
#     "Drone2": "anafi2",
# }

DRONE_TO_MODEL = {
    "Drone1" : "4k",
    "Drone2" : "usa",
    "Drone3": "thermal"
}

def wait_for_first_pose(node: PosePublisher):
    node.get_logger().info(f"[{node.drone_name}] Waiting for first pose...")
    while rclpy.ok() and node.get_pose() is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info(f"[{node.drone_name}] Got first pose!")

def wait_for_subscriber(node: PosePublisher):
    while rclpy.ok() and node.publisher.get_subscription_count() == 0:
        node.get_logger().info(f"[{node.drone_name}] Waiting for cmd subscriber...")
        rclpy.spin_once(node, timeout_sec=0.1)

def fly_mission(node: PosePublisher, altitude, drone_schedule, skills, objects):
    wait_for_first_pose(node)
    wait_for_subscriber(node)
    for subtask in drone_schedule:
        # print(f"Goal: {objects[subtask["object"]]}\nEx. time: {skills[subtask["skill"]]}\nObject: {subtask["object"]}\nSkill: {subtask["skill"]}\n", "="*90)
        node.move_and_execute(goal=objects[subtask["object"]], altitude=altitude, t=skills[subtask["skill"]], obj=subtask["object"], skill=subtask["skill"])
    node.get_logger().info(f"[{node.drone_name}]: Mission complete.")
    # print(f"{drone_name}: mission complete.")


if __name__ == "__main__":
    rclpy.init()

    NUM_DRONES = 3
    scanner = AnafiModelScanner(NUM_DRONES, DRONE_TO_MODEL)
    DRONE_TO_NODE = scanner.discover_models()
    print("\n=== ANAFI MODEL MAP ===")
    for drone, model in DRONE_TO_NODE.items():
        print(f"{drone}: {model}")

    # Create one node per drone and get its starting pose
    pose_publishers = {}
    flight_altitudes = {}

    try:
        for i, drone_name in enumerate(drones.keys()):
            # Create nodes for drones
            node = PosePublisher(drone_name, DRONE_TO_NODE)
            pose_publishers[drone_name] = node
            flight_altitudes[drone_name] = i + 1

            # Get start pos
            wait_for_first_pose(node)
            start_pose = node.get_pose()
            drones[drone_name]["pos"] = start_pose
            node.get_logger().info(f"[{drone_name}] Start pose: {start_pose}")

            # SET DRONE SPEED HERE
            node.set_speed(drones[drone_name]["speed"])

        # Plan mission
        task = ""  # DEFINE A TASK FOR THE NEW SKILLS 
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
                args=(node, flight_altitudes["drone_name"], drone_schedule, skills, objects),
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
