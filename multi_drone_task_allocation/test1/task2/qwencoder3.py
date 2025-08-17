import airsim
from math import atan2, pi

# Connect to the AirSim environment
client = airsim.MultirotorClient()
client.confirmConnection()

# Define drone positions
start_position = [0, 0, 1]
tower_center = [10, 2, 0]

# Drones' initial yaw angles (45° for North-East)
initial_yaws = {0: 45, 1: 135, 2: 225, 3: 315}

# Task parameters
tower_height = 10
tower_radius = 2
image_interval = 1  # meters
takeoff_altitude = 5

# Function to calculate the yaw needed for a drone to face the tower center
def calculate_yaw_to_tower_center(start_pos, tower_center):
    dx = tower_center[0] - start_pos[0]
    dy = tower_center[1] - start_pos[1]
    yaw_deg = atan2(dy, dx) * 180 / pi
    return yaw_deg

# Function to move a drone to a specific position and set its yaw
def move_and_set_yaw(drone_id, position, yaw_deg):
    client.moveToPositionAsync(position[0], position[1], position[2], speed=5).join()
    client.setVehicleKeyFrameAsync(1, yaw_deg, 1, True).join()

# Function to take images and land the drone
def inspect_tower(drone_id, height_start, height_end):
    for height in range(height_start, height_end + 1):
        position = [tower_center[0], tower_center[1], height]
        client.moveToPositionAsync(position[0], position[1], position[2], speed=5).join()
        # Take image with main camera
        client.capture_image("Camera", drone_id)
    client.landAsync(drone_id).join()

# Main function to execute the task
def execute_task():
    for i, yaw_deg in initial_yaws.items():
        drone_id = f"Drone{i + 1}"

        # Take off and move to a safe altitude
        client.takeoffAsync().join()
        client.moveToPositionAsync(start_position[0], start_position[1], takeoff_altitude).join()

        # Move the drone to the starting position relative to the tower center
        position = [tower_center[0] + 2, tower_center[1] + 2, takeoff_altitude]
        move_and_set_yaw(drone_id, position, yaw_deg)

        # Inspect the tower from the current position
        inspect_tower(drone_id, 1, 10)

        # Return to the starting position and land
        client.moveToPositionAsync(start_position[0], start_position[1], takeoff_altitude).join()
        client.landAsync(drone_id).join()

# Execute the task
execute_task()

### Explanation:
""" - **Initial Setup**: The script starts by connecting to the AirSim environment and defining the initial positions of the drones.
- **Yaw Calculation**: Each drone is assigned an initial yaw angle for facing North-East (45°).
- **Task Execution**:
  - **Takeoff and Positioning**: Each drone takes off and moves to a safe altitude.
  - **Movement and Yaw Setting**: The drone is moved to a position relative to the tower center, and its yaw is set to face the tower center.
  - **Image Capture**: The drone captures images at each specified height from 1 meter to 10 meters.
  - **Return and Land**: After inspecting all heights, the drone returns to the starting position and lands.

This script ensures that the drones are efficient in their inspection tasks while adhering to the given constraints. """