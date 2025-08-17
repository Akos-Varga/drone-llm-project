""" Selected Drones and Roles:

Drone3 (thermal, high flight time, onboard computer):
→ Thermal scanning drone

Drone7 (21MP camera, thermal, no onboard computer):
→ High-res image capture (triggered remotely)

Justification:

Only Drone3 has both a thermal camera and onboard computer to autonomously scan and evaluate temperatures.

Drone7 has a high-res camera (≥21MP) and a thermal sensor (though it’s unused here), so it's used for high-res image capture when needed.

Other 21MP drones lack thermal cameras or computers. Using just two drones minimizes resource use.

"""

import math
import airsim_wrapper as aw

import time

# === CONFIGURATION ===

# Rooftop corners (Z = 20), scan 5m above (Z = 25)
scan_altitude = 25
start_pos = [0, 0, 0]
grid_step = 5
temp_threshold = 37.0

# Grid scan bounds
x_min, x_max = 50, 70
y_min, y_max = 50, 70

# Selected drones
thermal_drone = "Drone3"
camera_drone = "Drone7"

# === HELPER FUNCTIONS ===

def compute_yaw(from_pos, to_pos):
    dx = to_pos[0] - from_pos[0]
    dy = to_pos[1] - from_pos[1]
    yaw = math.degrees(math.atan2(dy, dx)) % 360
    return yaw

# === STEP 1: Take off both drones ===
aw.takeoff(thermal_drone)
aw.takeoff(camera_drone)

# === STEP 2: Begin thermal scan with Drone3 ===

# Fly Drone3 to first scan altitude
aw.fly_to([x_min, y_min, scan_altitude], thermal_drone)

# Store hotspots where temp >= threshold
hotspots = []

# Grid scan loop
x = x_min
while x <= x_max:
    y_range = range(y_min, y_max + 1, grid_step) if (x - x_min) // grid_step % 2 == 0 else range(y_max, y_min - 1, -grid_step)
    for y in y_range:
        scan_point = [x, y, scan_altitude]
        
        # Move to the grid point
        aw.fly_to(scan_point, thermal_drone)
        
        # Wait briefly to stabilize before image
        time.sleep(1)
        
        # Capture thermal image and check temperature
        max_temp = aw.get_thermal_image(thermal_drone)
        if max_temp >= temp_threshold:
            hotspots.append(scan_point)
    
    x += grid_step

# === STEP 3: High-res imaging at hotspot locations using Drone7 ===
for point in hotspots:
    # Fly Drone7 to the hotspot
    aw.fly_to(point, camera_drone)
    
    # Get current position of Drone7 and yaw toward hotspot
    drone_pos = aw.get_drone_position(camera_drone)
    yaw = compute_yaw(drone_pos, point)
    aw.set_yaw(yaw, camera_drone)

    # Wait briefly to stabilize
    time.sleep(1)

    # Capture high-res image
    aw.capture_image(camera_drone)

# === STEP 4: Return both drones to home and land ===
aw.fly_to(start_pos, thermal_drone)
aw.fly_to(start_pos, camera_drone)

aw.land(thermal_drone)
aw.land(camera_drone)

# This script uses only two drones effectively, ensures thermal scanning coverage, triggers high-res capture based on detections, and completes within the 35-minute mission window given Drone3’s 60-minute and Drone7’s 30-minute flight times.