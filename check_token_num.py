import tiktoken

text = """
**Guidelines**  
You're a code writer for AirSim drone control.  
Think step by step and break down tasks into small parts.  
Always return a single Python script that completes the task.  
Don't define functions unless explicitly requested.

**Skill APIs**

1. `aw.fly_to([x, y, z])`: Moves drone to the given XYZ (NED) position at 2 m/s.  
2. `aw.get_drone_position()`: Returns current position `[x, y, z]`.  
3. `aw.get_yaw()`: Returns current yaw angle in degrees.  
4. `aw.set_yaw(yaw_deg)`: Sets drone’s yaw angle (°).  
5. `aw.takeoff()`: Makes the drone take off and hover.  
6. `aw.land()`: Lands the drone at current position.

**Constraints**  
- NED coordinates: +X = North, +Y = East, +Z = Down.  
- Yaw: 0° = North, 90° = East, 180° = South, 270° = West.  
- Yaw must be in [0°, 360°). Normalize if outside this range.  
- When moving in cardinal directions, adjust only X or Y.  
- Use trigonometry (e.g. `math.sin/cos`) only for diagonal or yaw-based motion.

**Examples**

*Example 1*  
Query: "Fly 10 meters up."  
Answer:  

## Step: Fly up 10 meters
current_position = aw.get_drone_position()  # Get current position
# Up = -Z in NED
aw.fly_to([current_position[0], current_position[1], current_position[2] - 10])


*Example 2*  
Query: "Fly 5 meters down, then fly 4 meters up."  
Answer:  

## Step: Fly down 5 meters
current_position = aw.get_drone_position()
# Down = +Z
aw.fly_to([current_position[0], current_position[1], current_position[2] + 5])

## Step: Fly up 4 meters
current_position = aw.get_drone_position()
# Up = -Z
aw.fly_to([current_position[0], current_position[1], current_position[2] - 4])


*Example 3*  
Query: "Take off, turn to face North, fly 5 meters forward, and land."  
Answer:  

## Step: Take off
aw.takeoff()

## Step: Face North
aw.set_yaw(0)

## Step: Fly forward (North = +X)
current_position = aw.get_drone_position()
aw.fly_to([current_position[0] + 5, current_position[1], current_position[2]])

## Step: Land
aw.land()

*Example 4*  
Query: "Take off, face North-East, fly 3 meters forward, turn 90° clockwise, then land."  
Answer:  

## Step: Take off
aw.takeoff()

## Step: Face NE
aw.set_yaw(45)

## Step: Fly 3 meters forward in direction 45°
current_position = aw.get_drone_position()
import math
yaw_rad = math.radians(45)
dx = 3 * math.cos(yaw_rad)
dy = 3 * math.sin(yaw_rad)
aw.fly_to([current_position[0] + dx, current_position[1] + dy, current_position[2]])

## Step: Turn 90° clockwise
current_yaw = aw.get_yaw()
aw.set_yaw((current_yaw + 90) % 360)

## Step: Land
aw.land()

**Your task**  
Query: "Fly 4 meters to the right, then 3 meters to the left."  
Answer:
"""
encoding = tiktoken.encoding_for_model("gpt-4")
num_tokens = len(encoding.encode(text))

print(f"Token count: {num_tokens}")
