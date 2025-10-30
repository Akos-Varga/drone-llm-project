from pipeline.decomposer import messages as decomposer_prompt
from pipeline.allocator import messages as allocator_prompt
from pipeline.scheduler import messages as scheduler_prompt

from test_tasks import task_list
from utils.travel_time import compute_travel_times
from utils.schedule_validator import validate_schedule
from utils.decomposer_validator import validate_decomposer
from utils.inference import LM
from utils.drone_visualizer import animate_schedule
from utils.randomizer import randomizer
from utils.vrp_allocator import solve_vrp
from utils.compare_schedules import schedules_equal

import matplotlib.pyplot as plt
from pprint import pprint
import pandas as pd
import time
import ast
import csv
import os

# Message builder --------------------------------------------------------------------------
def build_message(prompt, content):
     # print(content)
     return [*prompt, {"role": "user", "content": content}]

# Helpers  ---------------------------------------------------------------------------------
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
    
def append_row_csv(save, path, row, fieldnames):
    # ensure directory exists
    if save:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        write_header = not os.path.exists(path)
        with open(path, "a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if write_header:
                writer.writeheader()
            writer.writerow(row)

# Models -----------------------------------------------------------------------------------
m1 = "codegemma:7b"
m2 = "qwen2.5-coder:7b"
m3 = "gpt-4o-mini"
m4 = "gpt-4o"
m5 = "gpt-4.1-mini"
m6 = "gpt-4.1"
m7 = "gpt-5-mini"
m8 = "gpt-5"
models = [m6]

# Skills & Objects and & Drones ------------------------------------------------------------
skills = {
    'CaptureRGBImage': 1.1,
    'CaptureThermalImage': 2.4,
    'InspectStructure': 1.3,
    'MeasureWind': 2.7,
    'RecordVideo': 1.5
}

objects = {
    'Base': (75, 0, 53),
    'House1': (45, 70, 29),
    'House2': (78, 44, 53),
    'House3': (40, 40, 90),
    'RoofTop1': (4, 99, 39),
    'RoofTop2': (84, 37, 90),
    'SolarPanel1': (84, 82, 21),
    'SolarPanel2': (72, 8, 84),
    'Tower': (53, 91, 57)
}

drones = {
    'Drone1': {'pos': (34, 89, 13), 'skills': ['CaptureThermalImage'], 'speed': 19},
    'Drone2': {'pos': (89, 17, 9), 'skills': ['MeasureWind', 'CaptureRGBImage'], 'speed': 19},
    'Drone3': {'pos': (42, 72, 57), 'skills': ['CaptureRGBImage', 'CaptureThermalImage'], 'speed': 15},
    'Drone4': {'pos': (49, 87, 76), 'skills': ['CaptureRGBImage', 'RecordVideo'], 'speed': 13},
    'Drone5': {'pos': (64, 37, 95), 'skills': ['CaptureThermalImage', 'InspectStructure'], 'speed': 19},
    'Drone6': {'pos': (73, 98, 58), 'skills': ['MeasureWind', 'InspectStructure', 'RecordVideo'], 'speed': 11}
}

# skills, objects, drones = randomizer(skills=skills, objects=objects, drones=drones)

# Inference --------------------------------------------------------------------------------
save = False
if save:
    CSV_PATH = os.path.join("results", "test_results2.csv")
    FIELDNAMES = ["model", "task_id", "LLM_makespan", "VRP_makespan", "VRP_status", "LLM_inference_time", "LLM_error"]

for model in models:
    for task in task_list[7:]:
        print("="*90 + f"\n{task["id"]}: {task["task"]}")
        startTime = time.time()

        ## Decomposer
        decomposer_message = build_message(decomposer_prompt, f"task = {task['task']}\n\nskills = {skills}\n\nobjects = {objects}")
        # print(decomposer_message)
        decomposed_task_str = LM(model=model, messages=decomposer_message, printing=False)
        print(f"\n\nDecomposed task: {decomposed_task_str}")
        decomposed_task = str_to_code(decomposed_task_str)
        if not decomposed_task:
            error = "ERROR: decomposed_task conversion failed"
            print(f"\n\n{error}")
            row = {"model": model, "task_id": task["id"], "LLM_makespan": None, "VRP_makespan": None, "VRP_status": None, "LLM_inference_time": None, "LLM_error": error} # Save result than continue
            append_row_csv(save, CSV_PATH, row, FIELDNAMES)
            continue

        error = validate_decomposer(decomposed_task, task["solution"], skills)
        if error:
            print(f"\n\n{error}")
            row = {"model": model, "task_id": task["id"], "LLM_makespan": None, "VRP_makespan": None, "VRP_status": None, "LLM_inference_time": None, "LLM_error": error} # Save result than continue
            append_row_csv(save, CSV_PATH, row, FIELDNAMES)
            continue

        # Allocator
        allocator_message = build_message(allocator_prompt, f"drones = {drones}\n\nsubtasks = {decomposed_task}")
        # print(allocator_message)
        subtasks_with_drones_str = LM(model=model, messages=allocator_message, printing=False)
        print(f"\n\nAllocated task: {subtasks_with_drones_str}")
        subtasks_with_drones = str_to_code(subtasks_with_drones_str)
        if not subtasks_with_drones:
            error = "ERROR: subtasks_with_drones conversion failed"
            print(f"\n\n{error}")
            row = {"model": model, "task_id": task["id"], "LLM_makespan": None, "VRP_makespan": None, "VRP_status": None, "LLM_inference_time": None, "LLM_error": error} # Save result than continue
            append_row_csv(save, CSV_PATH, row, FIELDNAMES)
            continue
        travel_times = compute_travel_times(objects, drones, subtasks_with_drones)
        # pprint(travel_times, sort_dicts=False)

        # Scheduler
        scheduler_message = build_message(scheduler_prompt, f"subtasks_with_drones = {subtasks_with_drones_str}\n\ntravel_times = {travel_times}")
        # print(scheduler_message)
        schedule_str = LM(model=model, messages=scheduler_message, printing=False)
        print(f"\n\nScheduled task: {schedule_str}")
        schedule = str_to_code(schedule_str)
        if not schedule:
            error = "ERROR: schedule conversion failed"
            print(f"\n\n{error}")
            row = {"model": model, "task_id": task["id"], "LLM_makespan": None, "VRP_makespan": None, "VRP_status": None, "LLM_inference_time": None, "LLM_error": error} # Save result than continue
            append_row_csv(save, CSV_PATH, row, FIELDNAMES)
            continue
        error, makespan = validate_schedule(skills, objects, drones, subtasks_with_drones, travel_times, schedule)
        if error:
            print(f"\n\n{error}")
            row = {"model": model, "task_id": task["id"], "LLM_makespan": None, "VRP_makespan": None, "VRP_status": None, "LLM_inference_time": None, "LLM_error": error} # Save result than continue
            append_row_csv(save, CSV_PATH, row, FIELDNAMES)
            continue
        endTime = time.time()

        # Calculate with VRP (Vehicle Routing problem)
        schedule_vrp, makespan_vrp, status = solve_vrp(objects, drones, subtasks_with_drones)
        print("\n\nSchedule by VRP:")
        pprint(schedule_vrp, sort_dicts=False)
        print(f"\n\nSchedule is {status}")
        identical_schedule = schedules_equal(schedule, schedule_vrp)
        if identical_schedule:
            print("\n\nSchedules are identical.")
        else:
            print("\n\nSchedules are different.")
        inference_time = round(endTime - startTime, 1)
        print(f"\n\nMakespan:\n\tLLM: {makespan}\n\tVRP: {makespan_vrp}\nInference time: {inference_time}")

        # Save results
        row = {"model": model, "task_id": task["id"], "LLM_makespan": makespan, "VRP_makespan": makespan_vrp, "VRP_status": status, "LLM_inference_time": inference_time, "LLM_error": None} # Save result than continue
        append_row_csv(save, CSV_PATH, row, FIELDNAMES)

        # Visualize
        # anim = animate_schedule(objects, drones, schedule, dt=0.1, extra_hold=1.5, save_path=os.path.join("results", "animations", f"{task['id']}.gif"))
        # plt.show()

        # Update drone positions
        # for drone, info in schedule.items():
        #     if info:
        #         drones[drone]["pos"] = objects[info[-1]["object"]]
