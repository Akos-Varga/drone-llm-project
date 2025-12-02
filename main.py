import threading
import rclpy
import time
from rclpy.executors import MultiThreadedExecutor
from publisher import PosePublisher
# from model_discovery import AnafiModelScanner
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
        time.sleep(0.1)
    node.get_logger().info(f"[{node.drone_name}] Got first pose!")

def wait_for_subscriber(node: PosePublisher):
    node.get_logger().info(f"[{node.drone_name}] Waiting for cmd subscriber...")
    while rclpy.ok() and node.publisher.get_subscription_count() == 0:
        time.sleep(0.1)
    node.get_logger().info(f"[{node.drone_name}] Got subscriber!")

def fly_mission(node: PosePublisher, altitude, drone_schedule, skills, objects):
    wait_for_first_pose(node)
    wait_for_subscriber(node)
    node.arm()
    time.sleep(1)
    node.takeoff()
    time.sleep(5)
    node.offboard()
    for subtask in drone_schedule:
        node.move_and_execute(goal=objects[subtask['object']], altitude=altitude, t=skills[subtask['skill']], obj=subtask['object'], skill=subtask['skill'])
    node.get_logger().info(f"[{node.drone_name}]: Mission complete.")
    node.land()
    node.destroy_node()


if __name__ == "__main__":
    rclpy.init()

    DRONE_TO_NODE = {
        "Drone1": "anafi1",
        "Drone2": "anafi2",
    }

    MAX_ALTITUDE = 4.0

    pose_publishers = {}
    flight_altitudes = {}

    for i, drone_name in enumerate(drones.keys()):
        node = PosePublisher(drone_name, DRONE_TO_NODE)
        pose_publishers[drone_name] = node
        node.set_max_altitude(MAX_ALTITUDE)
        planned_altitude = i * 0.5 + 2.5
        if planned_altitude > MAX_ALTITUDE:
            raise ValueError(f"[{drone_name}] Planned altitude {planned_altitude:.2f} m exceeds max allowable altitude {MAX_ALTITUDE:.2f} m")
        flight_altitudes[drone_name] = planned_altitude
        node.set_speed(drones[drone_name]['speed'])

    # ---- Create executor and spin in a separate thread ----
    executor = MultiThreadedExecutor()

    for node in pose_publishers.values():
        executor.add_node(node)

    import threading

    def spin_executor():
        try:
            executor.spin()
        finally:
            for n in pose_publishers.values():
                executor.remove_node(n)

    spin_thread = threading.Thread(target=spin_executor, daemon=True)
    spin_thread.start()

    for drone_name, node in pose_publishers.items():
        wait_for_first_pose(node)

    # Plan mission
    task = "Record video of both wind turbines and capture thermal images of both towers."
    results = pipeline("gpt-5-mini", task, skills, objects, drones)
    schedule = results['schedule']

    threads = []

    for drone_name, drone_schedule in schedule.items():
        if not drone_schedule:
            continue

        node = pose_publishers[drone_name]

        t = threading.Thread(
            target=fly_mission,
            args=(node, flight_altitudes[drone_name], drone_schedule, skills, objects),
            daemon=True,
        )
        threads.append(t)
        t.start()

    for t in threads:
        t.join()

    # Cleanup
    for node in pose_publishers.values():
        node.destroy_node()

    executor.shutdown()
    rclpy.shutdown()
