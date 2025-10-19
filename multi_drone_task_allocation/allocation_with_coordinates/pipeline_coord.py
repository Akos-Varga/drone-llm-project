from decomposer_coord import messages as decomposer_prompt
from allocator_coord import messages as allocator_prompt
from scheduler_coord import messages as scheduler_prompt

from utils.test_tasks import task_list
from utils.travel_time import compute_travel_times
from utils.schedule_validator import ScheduleValidator
from utils.decomposer_validator import validate_decomposer
from utils.inference import LM
from utils.drone_visualizer import animate_schedule
from utils.randomizer import randomizer
from utils.vrp_allocator import solve_vrp
from utils.compare_schedules import schedules_equal

import matplotlib.pyplot as plt
from pprint import pprint
import ast
import time

# Message builder --------------------------------------------------------
def build_message(prompt, content):
     # print(content)
     return [*prompt, {"role": "user", "content": content}]

# Helpers  ---------------------------------------------------
def str_to_code(s):
    try:
      s = s.strip()
      if s.endswith("```"):
            s = s[:s.rfind("```")].strip()
      rhs = s.split('=',1)[1]
      rhs = "\n".join(line.split("#", 1)[0] for line in rhs.splitlines())
      # print(repr(rhs))

      return ast.literal_eval(rhs)
    except (SyntaxError, ValueError):
        return None

# Models -----------------------------------------------------------------
m1 = "qwen2.5-coder:1.5b"
m2 = "qwen2.5-coder:3b"
m3 = "codegemma:7b"
m4 = "qwen2.5-coder:7b"
m5 = "gpt-4o-mini"
m6 = "gpt-5-mini"

# Skills & Objects and & Drones ------------------------------------------------------------
skills = {
    "RecordVideo": 1.8,
    "CaptureRGBImage": 2.6,
    "CaptureThermalImage": 1.2,
    "MeasureWind": 2.9,
    "InspectStructure": 0.7
}

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

skills, objects, drones = randomizer(skills=skills, objects=objects, drones=drones)

# Inference --------------------------------------------------------------
model = m6
schedule_validator = ScheduleValidator(skills, objects, drones)

for task in task_list[6:8]:
    print("="*90 + f"\n{task["id"]}: {task["task"]}")
    startTime = time.time()

    ## Decomposer
    decomposer_message = build_message(decomposer_prompt, f"task = {task['task']}\n\nskills = {skills}\n\nobjects = {objects}")
    # print(decomposer_message)
    decomposed_task_str = LM(model=m5, messages=decomposer_message, printing=False)
    decomposed_task = str_to_code(decomposed_task_str)
    if decomposed_task == None:
        print(f"\n\nERROR: decomposed_task conversion failed\n\n{decomposed_task_str}")
        continue
    valid = validate_decomposer(decomposed_task, task["solution"], skills)
    if not valid:
        continue

    # Allocator
    allocator_message = build_message(allocator_prompt, f"drones = {drones}\n\nsubtasks = {decomposed_task}")
    # print(allocator_message)
    subtasks_with_drones_str = LM(model=m6, messages=allocator_message, printing=False)
    subtasks_with_drones = str_to_code(subtasks_with_drones_str)
    if subtasks_with_drones == None:
        print(f"\n\nERROR: subtasks_with_drones conversion failed\n\nDecomposed task: {decomposed_task_str}\n\nAllocated task: {subtasks_with_drones_str}")
        continue
    travel_times = compute_travel_times(objects, drones, subtasks_with_drones)
    # print(f"travel_times = {travel_times}\n\n")
    # pprint(travel_times, sort_dicts=False)

    # Scheduler
    scheduler_message = build_message(scheduler_prompt, f"subtasks_with_drones = {subtasks_with_drones_str}\n\ntravel_times = {travel_times}")
    # print(scheduler_message)
    schedule_str = LM(model=m6, messages=scheduler_message, printing=False)
    schedule = str_to_code(schedule_str)
    if schedule == None:
        print(f"\n\nERROR: Schedule conversion failed.\n\nDecomposed task: {decomposed_task_str}\n\nAllocated task: {subtasks_with_drones_str}\n\nScheduled task: {schedule_str}")
        continue
    valid, makespan = schedule_validator.validate_schedule(schedule, travel_times, subtasks_with_drones)
    if valid:
        print(f"\n\nMake span with LM: {makespan}")
    print(f"\n\nDecomposed task: {decomposed_task_str}\n\nAllocated task: {subtasks_with_drones_str}\n\nScheduled task: {schedule_str}")
    endTime = time.time()
    print(f"\n\nInference time: {(endTime - startTime):.1f}\n\n")

    # Calculate with VRP (Vehicle Routing problem)
    schedule_vrp, makespan_vrp = solve_vrp(subtasks_with_drones, drones, objects)
    print("Schedule by VRP:")
    pprint(schedule_vrp, sort_dicts=False)
    optimal_schedule = schedules_equal(schedule, schedule_vrp)
    if optimal_schedule:
        print("\n\nSchedules are identical.")
    else:
        print("\n\nSchedules are different.")

    # Visualize
    anim = animate_schedule(objects, drones, schedule, dt=0.1, extra_hold=1.5)
    plt.show()

    # Update drone positions
    # for drone, info in schedule.items():
    #     if info:
    #         drones[drone]["pos"] = objects[info[-1]["object"]]
