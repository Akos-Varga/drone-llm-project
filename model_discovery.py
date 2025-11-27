#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters


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
        node_name = f"{namespace}/anafi"    # e.g. anafi1/anafi
        srv_name = f"{node_name}/get_parameters"

        # Create client
        client = self.create_client(GetParameters, srv_name)

        self.get_logger().info(f"Checking parameter service {srv_name} ...")

        # Wait for service
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"{srv_name} not available.")
            return None

        # Prepare request
        req = GetParameters.Request()
        req.names = ["drone/model"]

        # Call service
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if not future.result():
            self.get_logger().warn(f"{node_name} returned no result.")
            return None

        result = future.result()

        if len(result.values) == 0:
            self.get_logger().warn(f"{node_name} has no drone/model parameter.")
            return None

        value = result.values[0].string_value
        self.get_logger().info(f"{node_name} → model '{value}'")
        return value

    def discover_models(self):
        """
        Returns a dict: {"thermal" : "anafi1", "usa" : "anafi2", ...}
        """

        mapping = {}

        for i in range(1, self.num_drones + 1):
            anafi_name = f"anafi{i}"
            model = self.get_model_for(anafi_name)

            if model is None:
                continue

            # Find matching symbolic drone name
            for drone_name, model_value in self.DRONE_TO_MODEL.items():
                if model_value == model:
                    mapping[drone_name] = anafi_name

        return mapping


def main():
    DRONE_TO_MODEL = {
        "Drone1": "4k",
        "Drone2": "usa",
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
