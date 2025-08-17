import openai
import time
from ollama import chat
from dotenv import load_dotenv
import os

load_dotenv()
api_key = os.getenv("OPENAI_API_KEY")
client = openai.OpenAI(api_key = api_key)

PLANNER_PROMPT = """
You are an expert in multi-drone task planning. Your job is to take a drone task described in natural language and return:
1. A high-level plan description in steps
2. An allocation of drones based on their capabilities
3. Python code using the imported `skills` functions to execute the task

Only use the listed API functions. Do not invent or define new ones.

Respond in the following format:
### Task description: ...

### Task decomposition:
- Step 1: ...
- Step 2: ...

### Drone Allocation:
- DroneX: ...
- DroneY: ...

### Executable Plan:
```python
# Python code below
...
```
"""

DRONE_SKILLS = """
Drone capabilities and control API functions:

- takeoff(drone_id): Drone takes off and hovers.
  Available on all drones

- land(drone_id): Drone lands at its current position.
  Available on all drones

- fly_to([x, y, z], drone_id): Moves the drone to a specified 3D coordinate.
  Available on all drones

- set_yaw(yaw_deg, drone_id): Rotates the drone to a specific yaw angle in degrees.
  Available on all drones

- capture_image(drone_id): Captures an image from the RGB camera.
  Available on all drones

- get_thermal_image(drone_id): Captures a thermal image and returns the max temperature.
  Available on: Drone3, Drone4, Drone6, Drone7

- get_drone_position(drone_id): Returns the current position [x, y, z].
  Available on all drones

- get_yaw(drone_id): Returns the current yaw (degrees).
  Available on all drones

- detect_object(object_name, image): Uses onboard computer to detect the specified object in an image.  
  Returns `True` if the object is found, otherwise `False`.  
  Available on: Drone1, Drone2, Drone3, Drone4. Other drones cannot detect objects.

- attach_payload(payload_weight_g, drone_id): Attaches a payload (in grams) to the drone.  
  Must be called **before takeoff**.
  Available on: Drone3, Drone4. Other drones cannot carry payloads.

- release_payload(drone_id): Releases the attached payload at the current location.
  Must be called **after land**.
  Available on: Drone3, Drone4. Other drones cannot carry payloads.
"""

EXAMPLE_TASK= """
TASK:

Scan a rooftop for heat signatures using thermal imaging from 5 meters above the rooftop.
Thermal images should be taken at every 5 meters in a grid pattern across the rooftop area.
If any location reports a maximum temperature of 37.0°C or higher, a second drone should capture a high-resolution image (≥21 MP) of that location from the same altitude.
All drones should return to the starting position and land when the task is completed.

ENVIRONMENT:
- Rooftop is a square with corners at [50, 50, 20], [70, 50, 20], [70, 70, 20], [50, 70, 20]
- Drones start at [0, 0, 0]
- Mission time: ~35 minutes
"""

EXAMPLE_SOLUTION = """
SOLUTION:

### Task decomposition:
- Step 1: A drone equipped with a thermal camera takes off and performs a thermal scan of the rooftop by flying to each point in a 5-meter grid at an altitude of 25 meters.
- Step 2: For each scanned point, if the detected temperature is ≥ 37.0°C, another drone with a high-resolution RGB camera flies to that point and captures an image.
- Step 3: After all tasks are complete, all drones return to the starting position and land.

### Drone Allocation:
- Drone3: has thermal camera
- Drone6: has 21 MP RGB camera

### Executable Plan:
```python
from skills import *
from math import cos, sin, atan2, pi, degrees

# === Step 1 ===
# A drone equipped with a thermal camera takes off and performs a thermal scan of the rooftop by flying to each point in a 5-meter grid at an altitude of 25 meters.

def generate_grid(corner1, corner2, spacing=5):
    x1, y1, z = corner1
    x2, y2, _ = corner2
    x_vals = list(range(int(x1), int(x2) + 1, spacing))
    y_vals = list(range(int(y1), int(y2) + 1, spacing))
    return [[x, y, z + 5] for x in x_vals for y in y_vals]  # 5m above rooftop

corner_start = [50, 50, 20]
corner_end = [70, 70, 20]
grid_points = generate_grid(corner_start, corner_end, spacing=5)

thermal_drone = "Drone3"  # has thermal camera
rgb_drone = "Drone6"      # has ≥21MP RGB camera

takeoff(thermal_drone)
takeoff(rgb_drone)

for point in grid_points:
    fly_to(point, thermal_drone)
    temp = get_thermal_image(thermal_drone)

    # === Step 2 ===
    # For each scanned point, if the detected temperature is ≥ 37.0°C, another drone with a high-resolution RGB camera flies to that point and captures an image.
    if temp >= 37.0:
        fly_to(point, rgb_drone)
        capture_image(rgb_drone)

# === Step 3 ===
# After all tasks are complete, all drones return to the starting position and land.
fly_to([0, 0, 5], thermal_drone)
fly_to([0, 0, 5], rgb_drone)

land(thermal_drone)
land(rgb_drone)
```
"""

def task_allocator(model, task):
    messages = [ 
        {"role": "system", "content": PLANNER_PROMPT},
        {"role": "system", "content": DRONE_SKILLS},
        #{"role": "user", "content": EXAMPLE_TASK},
        #{"role": "assistant", "content": EXAMPLE_SOLUTION},
        {"role": "user", "content": task}
    ]

    if("gpt" in model):
            response = client.chat.completions.create(
                model=model,
                messages=messages,
                temperature=0.0,
                max_tokens=1500
            )

            print(response.choices[0].message.content)
        
    else:
        response = chat(
          model=model,
          messages=messages,
          stream=True
        )

        output = ""
        for chunk in response:
            content = chunk.message.content
            print(content, end="", flush=True)
            output += content
        return output
##Prompts-----------------------------------------------------------------

t1 = f"""TASK:
A drone equipped with an RGB camera and onboard computer should fly along the street at an altitude of 23 meters, scanning at regular intervals for house structures.
If a house is detected, another drone capable of carrying at least 500 grams should deliver a small package to the detected location.
The delivery should only happen once.
All drones should return to the starting point and land when the mission is complete.

ENVIRONMENT:
- Street path: from [0, 0, 0] to [50, 0, 0], scan every 10 meters
- Package weight: 500 grams
- Drones start at [0, 0, 0]
- Mission time: ~12 minutes
"""

t2 = f"""TASK:
Inspect all four sides of a circular tower using 4 drones with onboard computers (one on each side).
Each drone should stay 2 meters away from the surface of the tower and take images every 1 meter in height from 1m to 10m.
Each drone should yaw to face the tower center while capturing images.
All drones should return to the starting position and land when the task is completed.

ENVIRONMENT:
- Tower height: 10 meters
- Tower base-center: [10, 2, 0]
- Tower radius: 2 meters
- Drones start at [0, 0, 0]
- Mission time: ~5 minutes
"""

##Models-----------------------------------------------------------------

m1 = "qwen2.5-coder:1.5b"
m2 = "qwen2.5-coder:3b"
m3 = "codegemma:7b"
m4 = "qwen2.5-coder:7b"
m5 = "gpt-3.5-turbo"

##Tests-----------------------------------------------------------------
tasks = [("Task 1", t1), ("Task 2", t2)]

for name, task in tasks:
    start = time.time()
    print(f"{name}\n")
    task_allocator(model=m1, task=task)
    end = time.time()
    print(f"\n--- Inference Time: {end - start:.2f} seconds ---\n" + "="*90)
