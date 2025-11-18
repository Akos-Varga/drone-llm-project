import threading
import rclpy
from ros_comm import PosePublisher
from world import objects, skills

def wait_for_first_pose(node: PosePublisher):
    node.get_logger().info(f"[{node.drone_prefix}] Waiting for first pose...")
    while rclpy.ok() and node.get_pose() is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info(f"[{node.drone_prefix}] Got first pose!")

def wait_for_subscriber(node: PosePublisher):
    while rclpy.ok() and node.publisher.get_subscription_count() == 0:
        node.get_logger().info(f"[{node.drone_prefix}] Waiting for cmd subscriber...")
        rclpy.spin_once(node, timeout_sec=0.1)

def fly_mission(node, drone, schedule, skills, objects):
    wait_for_first_pose(node)
    wait_for_subscriber(node)
    if schedule[drone] is not None:
        for subtask in schedule[drone]:
            # print(f"Goal: {objects[subtask["object"]]}\nEx. time: {skills[subtask["skill"]]}\nObject: {subtask["object"]}\nSkill: {subtask["skill"]}\n", "="*90)
            node.move_and_execute(objects[subtask["object"]], skills[subtask["skill"]], subtask["object"], subtask["skill"])
    node.get_logger().info(f"[{drone}]: Mission complete.")
    # print(f"{drone}: mission complete.")

    
schedule = {
    "Drone1": [
        {"name": "SubTask3", "object": "Base", "skill": "RecordVideo", "departure_time": 0.0, "arrival_time": 5.1, "finish_time": 5.6},
        {"name": "SubTask2", "object": "House3", "skill": "RecordVideo", "departure_time": 5.6, "arrival_time": 11.5, "finish_time": 12.0},
        {"name": "SubTask1", "object": "Tower", "skill": "RecordVideo", "departure_time": 12.0, "arrival_time": 17.8, "finish_time": 18.3}
    ],
    "Drone2": [
        {"name": "SubTask5", "object": "House3", "skill": "CaptureRGBImage", "departure_time": 0.0, "arrival_time": 3.3, "finish_time": 6.3},
        {"name": "SubTask4", "object": "Tower", "skill": "CaptureRGBImage", "departure_time": 6.3, "arrival_time": 10.2, "finish_time": 13.2}
    ],
    "Drone3": [
        {"name": "SubTask6", "object": "Base", "skill": "CaptureRGBImage", "departure_time": 0.0, "arrival_time": 6.9, "finish_time": 9.9}
    ],
    "Drone5": []
}

def main():
    rclpy.init()
    try:
        d1 = PosePublisher()
        # d2 = PosePublisher()

        t1 = threading.Thread(target=fly_mission, args=(d1, "Drone1", schedule, skills, objects), daemon=True)
        # t2 = threading.Thread(target=fly_mission, args=(d2, "Drone2", schedule, skills, objects), daemon=True)
        t1.start()
        # t2.start()
        t1.join()
        # t2.join()

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


if __name__ == "__main__":
    main()
