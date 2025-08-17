# Here is a Python script for controlling multiple drones using AirSim. The task involves scanning a rooftop for heat signatures using thermal imaging from 5 meters above the rooftop and capturing high-resolution images if needed.

import airsim

# Connect to AirSim
client = airsim.Client('127.0.0.1')

# List of available drones
drones = {
    "Drone1": {"id": 0, "flight_time_min": 30, "payload_g": 0, "has_computer": True, "main_camera_mp": 8, "has_thermal_camera": False},
    "Drone2": {"id": 1, "flight_time_min": 40, "payload_g": 0, "has_computer": True, "main_camera_mp": 12, "has_thermal_camera": False},
    "Drone3": {"id": 2, "flight_time_min": 60, "payload_g": 500, "has_computer": True, "main_camera_mp": 12, "has_thermal_camera": True},
    "Drone4": {"id": 3, "flight_time_min": 10, "payload_g": 500, "has_computer": True, "main_camera_mp": 4, "has_thermal_camera": True},
    "Drone5": {"id": 4, "flight_time_min": 25, "payload_g": 0, "has_computer": False, "main_camera_mp": 21, "has_thermal_camera": False},
    "Drone6": {"id": 5, "flight_time_min": 25, "payload_g": 0, "has_computer": false, "main_camera_mp": 21, "has_thermal_camera": True},
    "Drone7": {"id": 6, "flight_time_min": 30, "payload_g": 0, "has_computer": False, "main_camera_mp": 21, "has_thermal_camera": true},
    "Drone8": {"id": 7, "flight_time_min": 45, "payload_g": 0, "has_computer": false, "main_camera_mp": 48, "has_thermal_camera": false}
}

# Initialize drones
for drone_id, drone_info in drones.items():
    client.confirm_drone_connection(drone_id)
    client.enable_api_control(True, drone_id)

# Start position and target altitude
start_position = [0, 0, 2]
target_altitude = 5

# Function to scan the rooftop for heat signatures using thermal imaging
def scan_rooftop(drones):
    # Get initial positions of drones
    drone_positions = {drone_id: client.get_drone_position(drone_id) for drone_id in drones}

    # Grid pattern across the rooftop area
    grid_size = 10
    grid_step = 5
    rooftop_area = [[50, 50], [70, 50], [70, 70], [50, 70]]

    for i in range(grid_size):
        for j in range(grid_size):
            x1 = rooftop_area[i][0] + grid_step * i
            y1 = rooftop_area[i][1] + grid_step * j
            x2 = rooftop_area[i + 1][0] + grid_step * i
            y2 = rooftop_area[i + 1][1] + grid_step * j

            # Move to the center of the grid cell
            client.fly_to([x1 + (x2 - x1) / 2, y1 + (y2 - y1) / 2, target_altitude], drone_id)

            # Capture thermal image
            thermal_image = client.get_thermal_image(drone_id)
            max_temp = max(thermal_image[0])

            if max_temp >= 37.0:
                print(f"High temperature detected at [{x1 + (x2 - x1) / 2}, {y1 + (y2 - y1) / 2}] with max temp: {max_temp}°C")

                # Capture high-resolution image if needed
                client.takeoff(drone_id)
                client.move_to([x1 + (x2 - x1) / 2, y1 + (y2 - y1) / 2, target_altitude], drone_id)
                client.capture_image(drone_id)
                client.land(drone_id)

            else:
                print(f"No high temperature detected at [{x1 + (x2 - x1) / 2}, {y1 + (y2 - y1) / 2}]")

            # Return to start position
            client.fly_to(start_position, drone_id)

# Start the task
scan_rooftop(drones)

""" This script starts by connecting to AirSim and initializing all available drones. It then defines a function `scan_rooftop` that performs the following steps:

1. Get the initial positions of each drone.
2. Define a grid pattern across the rooftop area with specified step sizes.
3. Move each drone to the center of a grid cell, capture a thermal image, and check for high temperatures.
4. If any location reports a maximum temperature of 37.0°C or higher, it captures a high-resolution image from the same altitude.
5. Return each drone to its starting position.

The task is completed when all drones have returned to their starting positions and landed. """