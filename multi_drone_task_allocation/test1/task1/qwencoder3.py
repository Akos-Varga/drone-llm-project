import airsim

# Define the drone skills and available drones
drone_skills = {
    "Drone1": {"flight_time_min": 30, "payload_g": 0, "has_computer": True, "main_camera_mp": 8, "has_thermal_camera": False},
    "Drone2": {"flight_time_min": 40, "payload_g": 0, "has_computer": True, "main_camera_mp": 12, "has_thermal_camera": False},
    "Drone3": {"flight_time_min": 60, "payload_g": 500, "has_computer": True, "main_camera_mp": 12, "has_thermal_camera": true},
    "Drone4": {"flight_time_min": 10, "payload_g": 500, "has_computer": True, "main_camera_mp": 4, "has_thermal_camera": true},
    "Drone5": {"flight_time_min": 25, "payload_g": 0, "has_computer": False, "main_camera_mp": 21, "has_thermal_camera": false},
    "Drone6": {"flight_time_min": 25, "payload_g": 0, "has_computer": False, "main_camera_mp": 21, "has_thermal_camera": true},
    "Drone7": {"flight_time_min": 30, "payload_g": 0, "has_computer": false, "main_camera_mp": 21, "has_thermal_camera": true},
    "Drone8": {"flight_time_min": 45, "payload_g": 0, "has_computer": false, "main_camera_mp": 48, "has_thermal_camera": false}
}

# Select drones based on their skills
available_drones = [drone for drone in drone_skills if (
    drone_skills[drone]["flight_time_min"] >= 35 and
    drone_skills[drone]["payload_g"] > 0 and
    drone_skills[drone]["has_computer"]
)]

# Print selected drones and their assigned roles
print("Selected Drones:")
for drone in available_drones:
    print(f"{drone}: {drone_skills[drone]['flight_time_min']} minutes, Payload: {drone_skills[drone]['payload_g']} grams, Computer: {'Yes' if drone_skills[drone]['has_computer'] else 'No'}, Thermal Camera: {'Yes' if drone_skills[drone]['has_thermal_camera'] else 'No'}")

# Define the rooftop area
rooftop_area = [
    [50, 50, 20],  # North-West corner
    [70, 50, 20],  # South-West corner
    [70, 70, 20],  # South-East corner
    [50, 70, 20]   # North-East corner
]

# Drone IDs and their positions
drone_ids = available_drones

# Set the starting position for all drones
start_position = [0, 0, 0]
for drone in drone_ids:
    airsim.move_to_position(airsim.MultirotorClient(), start_position, drone)

# Initialize variables to track tasks
task_complete = False
second_drone_capture = False

# Task execution
while not task_complete:
    # Scan thermal images and check for hotspots
    for drone in drone_ids:
        if not second_drone_capture:  # Primary drone scans the area
            image_points = []
            for i in range(0, 51, 5):
                image_points.append([i, 50])
                image_points.append([70 - i, 50])
                image_points.append([70 - i, 70 - i])
                image_points.append([i, 70 - i])
            for point in image_points:
                airsim.move_to_position(airsim.MultirotorClient(), [point[0], point[1], 5], drone)
                thermal_image = airsim.get_thermal_image(drone)
                max_temp = thermal_image.max_temp_celsius
                if max_temp >= 37.0:
                    second_drone_capture = True
                    break
            # Return to the starting position after scanning
            airsim.move_to_position(airsim.MultirotorClient(), start_position, drone)
        else:  # Secondary drone captures high-resolution image of hotspots
            for point in image_points:
                airsim.move_to_position(airsim.MultirotorClient(), [point[0], point[1], 5], drone)
                if drone_skills[drone]['main_camera_mp'] >= 21:
                    airsim.capture_image(airsim.MultirotorClient(), "TopDown", drone)
            # Return to the starting position after capturing
            airsim.move_to_position(airsim.MultirotorClient(), start_position, drone)

    # End task once all drones have completed their tasks
    if second_drone_capture:
        task_complete = True

# Land all drones when the task is complete
for drone in drone_ids:
    airsim.land(drone)

print("Task Completed!")

""" **Explanation**:
1. **Selecting Drones**: The script first filters available drones based on their flight time, payload capacity, and computer presence.
2. **Drone IDs and Starting Positions**: It assigns these selected drones to specific IDs and sets them all to the starting position at `[0, 0, 0]`.
3. **Task Execution**:
   - The primary drone scans a grid pattern across the rooftop area, checking for hotspots using thermal imaging.
   - If a hotspot is detected (maximum temperature >= 37°C), a secondary drone captures a high-resolution image of that location from the same altitude.
4. **Completion Check**: The task is considered complete when all drones have either returned to their starting position or captured an image.
5. **Landing**: All drones land at their respective starting positions once the task is completed.

This script efficiently uses the available drones by dividing the scanning and capturing tasks, ensuring that only as many drones are needed as required to accomplish the mission within the given constraints. """