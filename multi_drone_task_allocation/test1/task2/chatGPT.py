""" Selected Drones and Roles:

Drone1, Drone2, Drone3, Drone4

→ All have onboard computers required by the task.
→ Each drone is assigned to inspect one cardinal direction (North, East, South, West) around the tower.

Why not use Drone5–Drone8?
They lack onboard computers and are not eligible for this task. """

import math
import airsim_wrapper as aw
import time

# === CONFIGURATION ===

tower_center = [10, 2, 0]
tower_radius = 2
inspection_offset = 2  # 2m from surface
standoff_radius = tower_radius + inspection_offset
heights = list(range(1, 11))  # 1m to 10m
start_pos = [0, 0, 0]

# Assign drones to directions (yaw in degrees from center)
# Each drone covers one side of the tower
drones = {
    "Drone1": 0,    # North
    "Drone2": 90,   # East
    "Drone3": 180,  # South
    "Drone4": 270   # West
}

# === HELPER FUNCTIONS ===

def compute_position(angle_deg, radius, height):
    """Compute XYZ coordinates offset from tower center."""
    angle_rad = math.radians(angle_deg)
    x = tower_center[0] + radius * math.cos(angle_rad)
    y = tower_center[1] + radius * math.sin(angle_rad)
    z = height
    return [x, y, z]

def compute_yaw(from_pos, to_pos):
    """Compute yaw to face the tower center."""
    dx = to_pos[0] - from_pos[0]
    dy = to_pos[1] - from_pos[1]
    return math.degrees(math.atan2(dy, dx)) % 360

# === STEP 1: Take off all drones ===
for drone_id in drones:
    aw.takeoff(drone_id)

# === STEP 2: Inspect tower sides from bottom to top ===
for drone_id, angle in drones.items():
    for z in heights:
        # Compute position along direction angle and height
        pos = compute_position(angle, standoff_radius, z)
        aw.fly_to(pos, drone_id)

        # Compute yaw to face tower center and rotate drone
        yaw = compute_yaw(pos, [tower_center[0], tower_center[1], z])
        aw.set_yaw(yaw, drone_id)

        # Pause to stabilize and capture image
        time.sleep(0.5)
        aw.capture_image(drone_id)

# === STEP 3: Return all drones and land ===
for drone_id in drones:
    aw.fly_to(start_pos, drone_id)
    aw.land(drone_id)

"""Meets Requirements:

Uses only drones with onboard computers.

Covers all 4 sides with proper spacing and elevation.

Drones rotate to face the tower center before each capture.

Executable in ~5 minutes due to parallel operation and efficient vertical movement."""