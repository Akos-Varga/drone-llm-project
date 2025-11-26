#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParametersClient


class AnafiModelScanner(Node):
    def __init__(self, num_drones: int, DRONE_TO_MODEL: dict):
        super().__init__("anafi_model_scanner")
        self.num_drones = num_drones
        self.DRONE_TO_MODEL = DRONE_TO_MODEL

    def get_model_for(self, namespace: str):
        """
        Read the drone/model parameter from /anafiX/anafi
        Returns string if found, else None.
        """
        node_name = f"{namespace}/anafi"      # e.g. /anafi1/anafi
        client = AsyncParametersClient(self, node_name)

        # Wait for parameter server
        self.get_logger().info(f"Checking {node_name} ...")
        for _ in range(20):
            if client.service_is_ready():
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            self.get_logger().warn(f"{node_name} parameter service not available.")
            return None

        future = client.get_parameters(["drone/model"])
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if not result or len(result.values) == 0:
            self.get_logger().warn(f"{node_name} has no drone/model parameter.")
            return None

        value = result.values[0].string_value
        self.get_logger().info(f"{node_name} → model '{value}'")
        return value

    def discover_models(self):
        """
        Returns a dict: {"thermal" : "anafi1": ,"usa" : "anafi2", ...}
        """

        mapping = {}

        for i in range(1, self.num_drones + 1):
            anafi_name = f"anafi{i}"
            model = self.get_model_for(anafi_name)
            for k, v in self.DRONE_TO_MODEL.items():
                if v == model:
                    drone = k
            mapping[drone] = anafi_name

        return mapping


def main():
    DRONE_TO_MODEL = {
        "Drone1" : "4k",
        "Drone2" : "usa",
        "Drone3": "thermal"
    }
    rclpy.init()

    NUM_DRONES = 4
    scanner = AnafiModelScanner(NUM_DRONES, DRONE_TO_MODEL)

    model_map = scanner.discover_models()
    print("\n=== ANAFI MODEL MAP ===")
    for drone, model in model_map.items():
        print(f"{drone}: {model}")

    scanner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
