import airsim as aw

# Define drone allocation
drone_roles = {
    "Drone1": "Image Capture",
    "Drone2": "Image Capture",
    "Drone3": "Image Capture",
    "Drone4": "Image Capture",
}

# Takeoff all drones
for drone_id in drone_roles.keys():
    aw.takeoff(drone_id)

# Define mission parameters
tower_height = 10
tower_base_center = [10, 2, 0]
tower_radius = 2
drone_start_position = [0, 0, 0]

# Define image capture parameters
image_height_interval = 1
image_height_start = 1
image_height_end = 10

# Function to get yaw angle to face the tower center
def get_yaw_to_tower(drone_position):
    dx = tower_base_center[0] - drone_position[0]
    dy = tower_base_center[1] - drone_position[1]
    return (180 / 3.14) * (atan2(dy, dx)) # !!! MATH not imported !!!

# Mission script
for drone_id in drone_roles.keys():
    # Move drone to starting position
    aw.fly_to(drone_start_position, drone_id)

    # Set yaw to face the tower center
    aw.set_yaw(get_yaw_to_tower(aw.get_drone_position(drone_id)), drone_id)

    # Iterate through heights and capture images
    for height in range(image_height_start, image_height_end + 1):
        # Move drone to image capture height
        aw.fly_to([ # !!! Drones fly to the center of the tower, not 4 sides !!!
            tower_base_center[0],
            tower_base_center[1],
            height + tower_base_center[2] - drone_start_position[2] ## !!! Just height is enough !!!
        ], drone_id)

        # Capture images
        aw.capture_image(drone_id) 

# Land all drones
for drone_id in drone_roles.keys():
    aw.land(drone_id) ## !!! No return to start position, just land at current position !!!