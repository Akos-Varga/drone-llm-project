from worlds.test_world import skills, objects, drones

from pipeline.decomposer import messages as decomposer_prompt
from pipeline.allocator import messages as allocator_prompt
from pipeline.scheduler import messages as scheduler_prompt

from test_tasks import task_list
from pipeline.utils.travel_time import compute_travel_times
from pipeline.utils.schedule_validator import validate_schedule
# from pipeline.utils.decomposer_validator import validate_decomposer
from pipeline.utils.inference import LM
# from pipeline.utils.drone_visualizer import animate_schedule
# from pipeline.utils.randomizer import randomizer
# from pipeline.utils.vrp_allocator import solve_vrp
# from pipeline.utils.compare_schedules import schedules_equal

# import matplotlib.pyplot as plt
from pprint import pprint
import time
import ast
import csv
import os

# Message builder --------------------------------------------------------------------------
def build_message(prompt, content):
     # print(content)
     return [*prompt, {'role': 'user', 'content': content}]

# Helpers  ---------------------------------------------------------------------------------
def str_to_code(s):
    try:
      s = s.strip()
      if s.endswith('```'):
          s = s[:s.rfind('```')].strip()
      rhs = s.split('=',1)[1]
      rhs = '\n'.join(line.split('#', 1)[0] for line in rhs.splitlines())
      # print(repr(rhs))

      return ast.literal_eval(rhs)
    except (SyntaxError, ValueError):
        return None
    
def append_row_csv(save, path, row, fieldnames):
    # ensure directory exists
    if save:
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        write_header = not os.path.exists(path)
        with open(path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if write_header: 
                writer.writeheader()
            writer.writerow(row)

# Inference --------------------------------------------------------------------------------
def pipeline(model, task, skills, objects, drones, solution = None, calculate_vrp = None):
    ## Decomposer
    decomposer_message = build_message(decomposer_prompt, f'task = {task}\n\nskills = {skills}\n\nobjects = {objects}')
    # print(decomposer_message)
    decomposed_task_str = LM(model=model, messages=decomposer_message, printing=False)
    print(f'\n\nDecomposed task: {decomposed_task_str}')
    decomposed_task = str_to_code(decomposed_task_str)
    if not decomposed_task: 
        return {'schedule': None, 'makespan': None, 'schedule_vrp': None, 'makespan_VRP': None, 'error': 'ERROR: decomposed_task conversion failed'}
    # if solution:
    #     error = validate_decomposer(decomposed_task, solution, skills)
    #     if error: 
    #         return {'schedule': None, 'makespan': None, 'schedule_vrp': None, 'makespan_VRP': None, 'error': error}
    
    # Allocator
    allocator_message = build_message(allocator_prompt, f'drones = {drones}\n\nsubtasks = {decomposed_task}')
    # print(allocator_message)
    subtasks_with_drones_str = LM(model=model, messages=allocator_message, printing=False)
    print(f'\n\nAllocated task: {subtasks_with_drones_str}')
    subtasks_with_drones = str_to_code(subtasks_with_drones_str)
    if not subtasks_with_drones: 
        return {'schedule': None, 'makespan': None, 'schedule_vrp': None, 'makespan_VRP': None, 'error': 'ERROR: subtasks_with_drones conversion failed'}
    travel_times = compute_travel_times(objects, drones, subtasks_with_drones)
    # pprint(travel_times, sort_dicts=False)

    # Scheduler
    scheduler_message = build_message(scheduler_prompt, f'subtasks_with_drones = {subtasks_with_drones_str}\n\ntravel_times = {travel_times}')
    # print(scheduler_message)
    schedule_str = LM(model=model, messages=scheduler_message, printing=False)
    print(f'\n\nScheduled task: {schedule_str}')
    schedule = str_to_code(schedule_str)
    if not schedule: 
        return {'schedule': None, 'makespan': None, 'schedule_vrp': None, 'makespan_VRP': None, 'error': 'ERROR: schedule conversion failed'}     
    error, makespan = validate_schedule(skills, objects, drones, subtasks_with_drones, travel_times, schedule)
    if error: 
        return {'schedule': None, 'makespan': None, 'schedule_vrp': None, 'makespan_VRP': None, 'error': error}
    return {'schedule': schedule, 'makespan': makespan, 'schedule_vrp': None, 'makespan_VRP': None, 'error': None}

    # Calculate with VRP (Vehicle Routing problem)
    # if calculate_vrp:
    #     schedule_vrp, makespan_vrp, status = solve_vrp(objects, drones, subtasks_with_drones)
    #     print('\n\nSchedule by VRP:')
    #     pprint(schedule_vrp, sort_dicts=False)
    #     print(f'\n\nSchedule is {status}')
    #     identical_schedule = schedules_equal(schedule, schedule_vrp)
    #     if identical_schedule: 
    #         print('\n\nSchedules are identical.')
    #     else: 
    #         print('\n\nSchedules are different.')
    #     print(f'\n\nMakespan:\n\tLLM: {makespan}\n\tVRP: {makespan_vrp}\n')
    #     # Save results
    #     return {'schedule': schedule, 'makespan': makespan, 'schedule_vrp': schedule_vrp, 'makespan_VRP': makespan_vrp, 'error': None}
    # else:
    #     return {'schedule': schedule, 'makespan': makespan, 'schedule_vrp': None, 'makespan_VRP': None, 'error': None}
# 
# 
if __name__ == '__main__':
    # Models -----------------------------------------------------------------------------------
    m1 = 'codegemma:7b'
    m2 = 'qwen2.5-coder:7b'
    m3 = 'gpt-4o-mini'
    m4 = 'gpt-4o'
    m5 = 'gpt-4.1-mini'
    m6 = 'gpt-4.1'
    m7 = 'gpt-5-mini'
    m8 = 'gpt-5'
    models = [m7]

    # skills, objects, drones = randomizer(skills=skills, objects=objects, drones=drones)

    save = False
    if save:
        CSV_PATH = os.path.join('results', 'test_results2.csv')
        FIELDNAMES = ['model', 'task_id', 'LLM_makespan', 'VRP_makespan', 'LLM_inference_time', 'LLM_error']

    for model in models:
        for task in task_list[:3]:
            print('='*90 + f"\n{task['id']}: {task['task']}")
            startTime = time.time()
            results = pipeline(model, task['task'], skills, objects, drones, task['solution'], calculate_vrp=True)
            endTime = time.time()
            inference_time = round(endTime - startTime, 1)
            if save:
                row = {'model': model, 'task_id': task['id'], 'LLM_makespan': results['makespan'], 'VRP_makespan': results['makespan_VRP'], 'LLM_inference_time': inference_time, 'LLM_error': results['error']}
                append_row_csv(save, CSV_PATH, row, FIELDNAMES)

            # Visualize
            # anim = animate_schedule(objects, drones, schedule, dt=0.1, extra_hold=1.5, save_path=os.path.join('results', 'animations', f'{task['id']}.gif'))
            # plt.show()
            # Update drone positions
            # for drone, info in schedule.items():
            #     if info:
            #         drones[drone]['pos'] = objects[info[-1]['object']]