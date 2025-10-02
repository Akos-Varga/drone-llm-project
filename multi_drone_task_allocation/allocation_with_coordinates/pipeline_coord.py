# python -m multi_drone_task_allocation.allocation_with_coordinates.pipeline_coord

from multi_drone_task_allocation.allocation_with_coordinates.decomposer_coord import messages as decomposer_prompt
from multi_drone_task_allocation.allocation_with_coordinates.allocator_coord import messages as allocator_prompt
from multi_drone_task_allocation.allocation_with_coordinates.scheduler_coord import messages as scheduler_prompt

from multi_drone_task_allocation.allocation_with_coordinates.test_tasks_coord import tasks
from multi_drone_task_allocation.allocation_with_coordinates.utils.travel_time import compute_travel_times
from multi_drone_task_allocation.allocation_with_coordinates.utils.schedule_validator import ScheduleValidator
from multi_drone_task_allocation.allocation_with_coordinates.utils.inference import LM

import time
import ast

# Message builder --------------------------------------------------------
def build_message(prompt, content):
     # print(content)
     return [*prompt, {"role": "user", "content": content}]

# Formatting functions ---------------------------------------------------
def remove_comments(commented_text):
     lines = commented_text.splitlines(keepends = True)
     return "".join(line for line in lines if not line.lstrip().startswith("#") and line.strip())

def str_to_code(str: str):
    rhs = str.split('=', 1)[1].strip()
    return ast.literal_eval(rhs)

# Models -----------------------------------------------------------------
m1 = "qwen2.5-coder:1.5b"
m2 = "qwen2.5-coder:3b"
m3 = "codegemma:7b"
m4 = "qwen2.5-coder:7b"
m5 = "gpt-4o-mini"
m6 = "gpt-5-mini"

# Actions & Objects and & Drones ------------------------------------------------------------
actions = ["RecordVideo", "CaptureRGBImage", "CaptureThermalImage", "MeasureWind", "InspectStructure"]

objects = {
    "Base": (18, 63),
    "RoofTop1": (72, 9),
    "RoofTop2": (41, 56),
    "SolarPanel1": (85, 22),
    "SolarPanel2": (87, 20),
    "House1": (5, 44),
    "House2": (92, 71),
    "House3": (47, 36),
    "Tower": (14, 7)
}

drones = {
  "Drone1": {"skills": ["CaptureThermalImage"], "pos": (27, 81), "speed": 15},
  "Drone2": {"skills": ["MeasureWind","CaptureRGBImage"], "pos": (63, 14), "speed": 18},
  "Drone3": {"skills": ["CaptureRGBImage", "CaptureThermalImage"], "pos": (92, 47), "speed": 12},
  "Drone4": {"skills": ["CaptureRGBImage", "RecordVideo"], "pos": (39, 59), "speed": 19},
  "Drone5": {"skills": ["CaptureThermalImage", "InspectStructure"], "pos": (8, 23), "speed": 11},
  "Drone6": {"skills": ["MeasureWind"], "pos": (74, 66), "speed": 16}
}

# Inference --------------------------------------------------------------
model = m6
schedule_validator = ScheduleValidator(objects, drones)
for task_id, user_task in list(tasks.items()):
  print(task_id + ": " + user_task)

  ## Decomposer
  decomposer_message = build_message(decomposer_prompt, f"task = {user_task}\n\nactions = {actions}\n\nobjects = {objects}")
  # print(decomposer_message)
  start_time = time.time()
  decomposed_task = LM(model=model, messages=decomposer_message)
  decomposed_task = remove_comments(decomposed_task)
  end_time = time.time()
  print(f"\n--- Inference Time: {end_time - start_time:.2f} seconds ---\n" + "="*90)

  # Allocator
  allocator_message = build_message(allocator_prompt, f"drones = {drones}\n\nsubtasks = {decomposed_task}")
  # print(allocator_message)
  start_time = time.time()
  subtasks_with_drones_str = LM(model=model, messages=allocator_message)
  subtasks_with_drones_str = remove_comments(subtasks_with_drones_str)
  subtasks_with_drones = str_to_code(subtasks_with_drones_str)
  travel_times = compute_travel_times(objects, drones, subtasks_with_drones)
  # print(f"travel_times = {travel_times}\n\n")
  end_time = time.time()
  print(f"\n--- Inference Time: {end_time - start_time:.2f} seconds ---\n" + "="*90)

  # Scheduler
  scheduler_message = build_message(scheduler_prompt, f"subtasks_with_drones = {subtasks_with_drones_str}\n\ntravel_times = {travel_times}")
  # print(allocator_message)
  start_time = time.time()
  schedule_str = LM(model=model, messages=scheduler_message)
  schedule_str = remove_comments(schedule_str)
  schedule = str_to_code(schedule_str)
  end_time = time.time()
  print(f"\n--- Inference Time: {end_time - start_time:.2f} seconds ---\n" + "="*90)
