from ollama import chat
import time
import openai
from dotenv import load_dotenv
import os

load_dotenv()
api_key = os.getenv("OPENAI_API_KEY")
client = openai.OpenAI(api_key = api_key)

def task_allocator(model, task):
    messages = [ 
        {
            "role": "system",
            "content": """
**Guidelines**
Write a Python script for controlling multiple drones using the already defined APIs below.
Allocate tasks to drones based on their capabilities (see Drone Skills).
Before the script, list the selected drones and their assigned roles, along with a short reason for each selection.
Use the minimum number of drones possible to complete the task.
Break tasks into sequential steps and include **inline comments** for each step in the code.

**Drone Skills**
- flight_time_min: Maximum flight time of the drone in minutes
- payload_g: Maximum payload capacity of the drone in grams
- has_computer: Whether the drone has an onboard computer
- main_camera_mp: Resolution of the main camera in megapixels
- has_thermal_camera: Whether the drone has a thermal camera

**Available Drones**
  - drone_id: Drone1
    flight_time_min: 30
    payload_g: 0
    has_computer: true
    main_camera_mp: 8
    has_thermal_camera: false

  - drone_id: Drone2
    flight_time_min: 40
    payload_g: 0
    has_computer: true
    main_camera_mp: 12
    has_thermal_camera: false

  - drone_id: Drone3
    flight_time_min: 60
    payload_g: 500
    has_computer: true
    main_camera_mp: 12
    has_thermal_camera: true

  - drone_id: Drone4
    flight_time_min: 10
    payload_g: 500
    has_computer: true
    main_camera_mp: 4
    has_thermal_camera: true

  - drone_id: Drone5
    flight_time_min: 25
    payload_g: 0
    has_computer: false
    main_camera_mp: 21
    has_thermal_camera: false

  - drone_id: Drone6
    flight_time_min: 25
    payload_g: 0
    has_computer: false
    main_camera_mp: 21
    has_thermal_camera: true

  - drone_id: Drone7
    flight_time_min: 30
    payload_g: 0
    has_computer: false
    main_camera_mp: 21
    has_thermal_camera: true

  - drone_id: Drone8
    flight_time_min: 45
    payload_g: 0
    has_computer: false
    main_camera_mp: 48
    has_thermal_camera: false

**APIs**
-- takeoff(drone_id): Drone takes off and hovers.

- land(drone_id): Drone lands at its current position.

- fly_to([x, y, z], drone_id): Moves the drone to a specified 3D coordinate.

- set_yaw(yaw_deg, drone_id): Rotates the drone to a specific yaw angle in degrees.

- capture_image(drone_id): Captures an image from the RGB camera.

- get_thermal_image(drone_id): Captures a thermal image and returns the max temperature.

- get_drone_position(drone_id): Returns the current position [x, y, z].

- get_yaw(drone_id): Returns the current yaw (degrees).

- detect_object(object_name, image): Uses onboard computer to detect the specified object in an image.  
  Returns `True` if the object is found, otherwise `False`.  

- attach_payload(payload_weight_g, drone_id): Attaches a payload (in grams) to the drone.  
  Must be called **before takeoff**.

- release_payload(drone_id): Releases the attached payload at the current location.
  Must be called **after land**.

**Constraints**
XYZ coordinate system:
+X = North
+Y = East
+Z = Up

Yaw convention:
0° = North
90° = East
180° = South
270° = West
Clockwise turn = positive yaw
Counter-clockwise turn = negative yaw
Normalize yaw to [0°, 360°)
Initial yaw = 0°
Use atan2(dy, dx) to compute yaw from drone to a target point.

Diagonal directions:
North-East = 45°
South-East = 135°
South-West = 225°
North-West = 315°
"""
        },
        {"role": "user",
         "content": f"""
**Your Task**
Query: "{task}"

Answer:

"""}]

    if("gpt" in model):
        response = client.chat.completions.create(
            model=model,
            messages=messages,
            temperature=0,
            max_tokens=1500
        )

        return response.choices[0].message.content
    
    else:
        response = chat(
            model=model,
            messages=messages,
            stream=False
        )

        return response["message"]["content"]

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
    result = task_allocator(model=m1, task=task)
    end = time.time()
    print(f"{name}\n{result}\n--- Inference Time: {end - start:.2f} seconds ---\n" + "="*90)
