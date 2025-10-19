from typing import Dict, List, Tuple
from math import sqrt
from ortools.sat.python import cp_model

# -------------------------
# Input data (from the user)
# -------------------------
subtasks_with_drones = [
    {"name": "SubTask1", "skill": "InspectStructure", "object": "House2", "service_time": 0.7, "drones": ["Drone5"]},
    {"name": "SubTask2", "skill": "MeasureWind", "object": "Base", "service_time": 2.9, "drones": ["Drone2", "Drone6"]},
    {"name": "SubTask3", "skill": "RecordVideo", "object": "Tower", "service_time": 1.8, "drones": ["Drone4"]},
]

objects = {
    "Base": (18, 63),
    "RoofTop1": (72, 9),
    "RoofTop2": (41, 56),
    "SolarPanel1": (85, 22),
    "SolarPanel2": (87, 20),
    "House1": (5, 44),
    "House2": (92, 71),
    "House3": (47, 36),
    "Tower": (14, 7),
}

drones = {
    "Drone1": {"skills": ["CaptureThermalImage"], "pos": (27, 81), "speed": 15},
    "Drone2": {"skills": ["MeasureWind", "CaptureRGBImage"], "pos": (63, 14), "speed": 18},
    "Drone3": {"skills": ["CaptureRGBImage", "CaptureThermalImage"], "pos": (92, 47), "speed": 12},
    "Drone4": {"skills": ["CaptureRGBImage", "RecordVideo"], "pos": (39, 59), "speed": 19},
    "Drone5": {"skills": ["CaptureThermalImage", "InspectStructure"], "pos": (8, 23), "speed": 11},
    "Drone6": {"skills": ["MeasureWind"], "pos": (74, 66), "speed": 16},
}

# -------------------------
# Helper functions
# -------------------------

def euclid(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def travel_time_from_to(drone_speed: float, a_xy: Tuple[float, float], b_xy: Tuple[float, float]) -> float:
    return euclid(a_xy, b_xy) / drone_speed


# -------------------------
# CP-SAT model (assignment + ordering per drone with travel times)
# NOTE: CP-SAT does not support continuous decision variables. We time-scale
# everything to integers (tenths of a time unit) using TIME_SCALE=10.
# -------------------------

def solve_vrp(subtasks: List[Dict], drones_data: Dict[str, Dict], objects: Dict[str, Tuple[int, int]]):
    model = cp_model.CpModel()

    TIME_SCALE = 10  # one decimal resolution -> multiply all times by 10

    tasks = [t["name"] for t in subtasks]
    drone_list = list(drones_data.keys())

    # Eligible drones per task (provided by user input)
    eligible = {t["name"]: list(t["drones"]) for t in subtasks}

    # Convenience maps
    service = {t["name"]: int(round(float(t["service_time"]) * TIME_SCALE)) for t in subtasks}
    obj_of = {t["name"]: t["object"] for t in subtasks}
    skill_of = {t["name"]: t["skill"] for t in subtasks}

    # Precompute travel times (scaled to int):
    start_travel = {}  # (d, t) -> int time
    travel = {}        # (d, i, j) -> int time (i != j)

    # Track a loose horizon upper bound
    horizon_bound = 0

    for d in drone_list:
        dpos = drones_data[d]["pos"]
        spd = float(drones_data[d]["speed"])
        for t in tasks:
            if d in eligible[t]:
                tpos = objects[obj_of[t]]
                arr = travel_time_from_to(spd, dpos, tpos)
                val = int(round(arr * TIME_SCALE))
                start_travel[(d, t)] = val
                horizon_bound = max(horizon_bound, val)
        for i in tasks:
            for j in tasks:
                if i == j:
                    continue
                if (d in eligible[i]) and (d in eligible[j]):
                    ipos = objects[obj_of[i]]
                    jpos = objects[obj_of[j]]
                    tij = travel_time_from_to(spd, ipos, jpos)
                    val = int(round(tij * TIME_SCALE))
                    travel[(d, i, j)] = val
                    horizon_bound = max(horizon_bound, val)

    # A very safe horizon: sum of all start_travel + all services + all pairwise traveltimes
    HORIZON = max(100000, 5 * horizon_bound + sum(service.values()) + 100)

    # Decision variables
    x = {}
    for t in tasks:
        for d in eligible[t]:
            x[(d, t)] = model.NewBoolVar(f"x_{d}_{t}")

    start = {t: model.NewIntVar(0, HORIZON, f"start_{t}") for t in tasks}
    finish = {t: model.NewIntVar(0, HORIZON, f"finish_{t}") for t in tasks}

    # Each task is assigned to exactly one eligible drone
    for t in tasks:
        model.Add(sum(x[(d, t)] for d in eligible[t]) == 1)

    # Finish = start + service
    for t in tasks:
        model.Add(finish[t] == start[t] + service[t])

    # First-task arrival constraints
    for d in drone_list:
        Td = [t for t in tasks if d in eligible[t]]
        for i in Td:
            if (d, i) in start_travel:
                model.Add(start[i] >= start_travel[(d, i)]).OnlyEnforceIf(x[(d, i)])

    # Pairwise precedences with sequence-dependent travel
    bigM = HORIZON
    for d in drone_list:
        Td = [t for t in tasks if d in eligible[t]]
        for i in Td:
            for j in Td:
                if i == j:
                    continue
                both = model.NewBoolVar(f"both_{d}_{i}_{j}")
                model.Add(both <= x[(d, i)])
                model.Add(both <= x[(d, j)])
                model.Add(both >= x[(d, i)] + x[(d, j)] - 1)

                y = model.NewBoolVar(f"y_{d}_{i}_before_{j}")
                tij = travel.get((d, i, j), 0)
                tji = travel.get((d, j, i), 0)
                # When both==1, either i before j or j before i
                model.Add(start[j] >= finish[i] + tij - bigM * (1 - y)).OnlyEnforceIf(both)
                model.Add(start[i] >= finish[j] + tji - bigM * y).OnlyEnforceIf(both)

    # Makespan
    makespan = model.NewIntVar(0, HORIZON, "makespan")
    for t in tasks:
        model.Add(makespan >= finish[t])
    model.Minimize(1000000 * makespan + sum(start[t] for t in tasks))

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 20.0
    solver.parameters.num_search_workers = 8

    status = solver.Solve(model)
    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        raise RuntimeError("No feasible schedule found.")

    # Recover assignments per drone and order tasks by start time
    schedule: Dict[str, List[Dict]] = {d: [] for d in drone_list}

    chosen_drone: Dict[str, str] = {}
    for t in tasks:
        for d in eligible[t]:
            if solver.Value(x[(d, t)]) == 1:
                chosen_drone[t] = d
                break

    for t in tasks:
        d = chosen_drone[t]
        arr = solver.Value(start[t]) / TIME_SCALE
        fin = solver.Value(finish[t]) / TIME_SCALE
        schedule[d].append({
            "name": t,
            "object": obj_of[t],
            "skill": skill_of[t],
            "departure_time": None,  # fill below
            "arrival_time": round(arr, 1),
            "finish_time": round(fin, 1),
        })

    # Post-process to compute departure times
    for d, lst in schedule.items():
        if not lst:
            continue
        lst.sort(key=lambda r: r["arrival_time"])  # sort by arrival
        lst[0]["departure_time"] = 0.0
        for k in range(1, len(lst)):
            prev = lst[k - 1]
            lst[k]["departure_time"] = prev["finish_time"]

    solved_makespan = max((r["finish_time"] for lst in schedule.values() for r in lst), default=0.0)

    return schedule, solved_makespan


if __name__ == "__main__":
    schedule, ms = solve_vrp(subtasks_with_drones, drones, objects)
    print("Solved multi-task schedule (min. makespan):")
    for d, lst in schedule.items():
        print(f"  {d}: {lst}")
    print(f"Makespan = {ms}")
