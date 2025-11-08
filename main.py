from main_pipeline import pipeline, append_row_csv
from world import skills, objects, drones
from test_tasks import task_list
import time
import os

# Models -----------------------------------------------------------------------------------
m1 = "codegemma:7b"
m2 = "qwen2.5-coder:7b"
m3 = "gpt-4o-mini"
m4 = "gpt-4o"
m5 = "gpt-4.1-mini"
m6 = "gpt-4.1"
m7 = "gpt-5-mini"
m8 = "gpt-5"
models = [m7]


# skills, objects, drones = randomizer(skills=skills, objects=objects, drones=drones)

save = False
if save:
    CSV_PATH = os.path.join("results", "test_results2.csv")
    FIELDNAMES = ["model", "task_id", "LLM_makespan", "VRP_makespan", "LLM_inference_time", "LLM_error"]

for model in models:
    for task in task_list[:3]:
        print("="*90 + f"\n{task["id"]}: {task["task"]}")
        startTime = time.time()
        results = pipeline(model, task, skills, objects, drones)
        endTime = time.time()
        inference_time = round(endTime - startTime, 1)
        if save:
            row = {"model": model, "task_id": task["id"], "LLM_makespan": results["makespan"], "VRP_makespan": results["makespan_VRP"], "LLM_inference_time": inference_time, "LLM_error": results["error"]}
            append_row_csv(save, CSV_PATH, row, FIELDNAMES)

        # Visualize
        # anim = animate_schedule(objects, drones, schedule, dt=0.1, extra_hold=1.5, save_path=os.path.join("results", "animations", f"{task['id']}.gif"))
        # plt.show()
        # Update drone positions
        # for drone, info in schedule.items():
        #     if info:
        #         drones[drone]["pos"] = objects[info[-1]["object"]]