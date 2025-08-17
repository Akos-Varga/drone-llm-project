# Import necessary libraries
import airsim as aw

# Define the rooftop dimensions
rooftop_size = 20

# Define the thermal image threshold
thermal_threshold = 37.0

# Define the drone allocation
drone1 = "Drone1"  # Flight time and camera !!! WRONG DRONE SELECTION !!!
drone2 = "Drone2"  # High-resolution camera !!! WRONG DRONE SELECTION !!!

# Task: Scan rooftop for heat signatures using thermal imaging
# 1. Take off and move to the starting position
aw.takeoff(drone1)
aw.fly_to([0, 0, 5], drone1)

# 2. Create a grid of points for thermal imaging
grid_size = 5
for x in range(0, rooftop_size, grid_size):
    for y in range(0, rooftop_size, grid_size):
        # 3. Fly to the grid point
        aw.fly_to([x + 50, y + 50, 5], drone1) # !!! WRONG ALTITUDE SELECTION !!!

        # 4. Capture thermal image and check for threshold
        temperature = aw.get_thermal_image(drone1)
        if temperature >= thermal_threshold:
            # 5. Take high-resolution image
            aw.fly_to([x + 50, y + 50, 5], drone2) # !!! WRONG ALTITUDE SELECTION !!!
            aw.capture_image(drone2)

# 6. Return to starting position
aw.fly_to([0, 0, 0], drone1) # !!! NO RETURN WITH DRONE2 !!!

# 7. Land all drones
aw.land(drone1)
aw.land(drone2)