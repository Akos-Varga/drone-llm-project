from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def solve_vrp(subtasks_with_drones, travel_times, time_limit_s=5):
    """
    Solve multi-drone scheduling with sequence-dependent travel times,
    per-task service times, drone eligibility, and makespan minimization.

    Inputs
    ------
    subtasks_with_drones: [
        {"name": str, "skill": str, "object": str, "service_time": float, "drones": [str, ...]}, ...
    ]
    travel_times: {
      "drone_to_object": {Drone: {Object: float}},
      "drone_object_to_object": {Drone: {FromObj: {ToObj: float}}}
    }

    Output
    ------
    schedule: {
      DroneName: [
        {"name": str, "object": str, "skill": str,
         "departure_time": float, "arrival_time": float, "finish_time": float}, ...
      ],
      ...
    }
    """

    # ---------- 1) Normalize basic data ----------
    tasks = [
        (t["name"], t["object"], t["skill"], float(t.get("service_time", 0.0)), list(t["drones"]))
        for t in subtasks_with_drones
    ]
    task_count = len(tasks)
    # All drones that appear in eligibility sets
    drones = sorted(set(d for *_, ds in tasks for d in ds))
    num_vehicles = len(drones)

    # Create one dedicated depot (start=end) node per vehicle
    # Global node indices: 0..task_count-1 = tasks, task_count..task_count+num_vehicles-1 = depots
    starts = [task_count + v for v in range(num_vehicles)]
    ends   = [task_count + v for v in range(num_vehicles)]

    # Quick helpers to access data
    def svc_time(node):
        return tasks[node][3] if 0 <= node < task_count else 0.0

    def travel_from_to(drone, from_node, to_node):
        """Pure travel time (no service). Depots have index >= task_count."""
        depot = task_count + drones.index(drone)
        # depot -> task
        if from_node == depot and to_node < task_count:
            obj = tasks[to_node][1]
            return float(travel_times["drone_to_object"][drone][obj])
        # task -> depot (we won't force a return travel; set 0 here)
        if from_node < task_count and to_node == depot:
            return 0.0
        # task -> task
        if from_node < task_count and to_node < task_count:
            from_obj = tasks[from_node][1]
            to_obj   = tasks[to_node][1]
            return float(travel_times["drone_object_to_object"][drone][from_obj][to_obj])
        # depot -> depot
        return 0.0

    # We model time as: transit(from→to) = service_time(at from) + travel_time(from→to)
    # This counts each task's service exactly once (when leaving the task).
    # For the first leg (depot->task), depot service is 0, as desired.
    def leg_time(drone, from_node, to_node):
        base = travel_from_to(drone, from_node, to_node)
        if 0 <= from_node < task_count:
            base += svc_time(from_node)
        return base

    # ---------- 2) OR-Tools Manager & Routing ----------
    manager = pywrapcp.RoutingIndexManager(task_count + num_vehicles, num_vehicles, starts, ends)
    routing = pywrapcp.RoutingModel(manager)

    # Vehicle-dependent transit callbacks (heterogeneous travel per drone)
    transit_cids = []
    for v, drone in enumerate(drones):
        def make_cb(drone_name):
            def cb(from_index, to_index):
                from_node = manager.IndexToNode(from_index)
                to_node   = manager.IndexToNode(to_index)
                # OR-Tools expects integers; scale by 1000 to keep ms resolution
                return int(round(1000.0 * leg_time(drone_name, from_node, to_node)))
            return cb
        cid = routing.RegisterTransitCallback(make_cb(drone))
        # Set arc cost for this vehicle to its own callback
        routing.SetArcCostEvaluatorOfVehicle(cid, v)
        transit_cids.append(cid)

    # ---------- 3) Enforce drone eligibility per task ----------
    for task_i, (*_, allowed) in enumerate(tasks):
        # Node index in routing
        node_index = manager.NodeToIndex(task_i)
        # Remove disallowed vehicles for this node
        for v, drone in enumerate(drones):
            if drone not in allowed:
                routing.VehicleVar(node_index).RemoveValue(v)

    # ---------- 4) Time dimension & makespan objective ----------
    routing.AddDimensionWithVehicleTransits(
        transit_cids,   # per-vehicle transit evaluators
        0,              # no waiting/slack
        10**12,         # large horizon
        True,           # force cumul at start to 0
        "Time"
    )
    time_dim = routing.GetDimensionOrDie("Time")

    # Minimize maximum route end (makespan). GlobalSpan is end_of_route - start (start=0 here).
    time_dim.SetGlobalSpanCostCoefficient(1)

    # Optional: disallow empty routes for vehicles not eligible to any task (kept flexible)
    # (Default is fine: vehicles can be unused.)

    # ---------- 5) First solution & local search ----------
    search = pywrapcp.DefaultRoutingSearchParameters()
    search.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search.time_limit.FromSeconds(int(time_limit_s))

    solution = routing.SolveWithParameters(search)
    if solution is None:
        raise RuntimeError("No feasible schedule found.")

    # ---------- 6) Extract schedule ----------
    schedule = {}
    time_scale = 1000.0  # we scaled to milliseconds
    makespan = 0 # max over all finish_time values

    for v, drone in enumerate(drones):
        idx = routing.Start(v)
        if routing.IsEnd(idx):
            continue  # vehicle unused
        seq = []
        prev_finish = 0.0  # departure time from depot is 0 by construction

        while not routing.IsEnd(idx):
            node = manager.IndexToNode(idx)
            next_idx = solution.Value(routing.NextVar(idx))

            if 0 <= node < task_count:
                name, obj, skill, stime, _ = tasks[node]

                # Arrival at this node equals the cumul at this node:
                arrival_here = solution.Value(time_dim.CumulVar(idx)) / time_scale
                finish_here  = arrival_here + stime

                # Departure from this node equals its finish time
                # (since leg_time adds service time at 'node' to the arc leaving it).
                # For the first task, prev_finish should match 0 for depot.
                # We overwrite departure_time with prev_finish to make it explicit.
                seq.append({
                    "name": name,
                    "object": obj,
                    "skill": skill,
                    "departure_time": float(prev_finish),
                    "arrival_time": float(arrival_here),
                    "finish_time": float(finish_here),
                })
                prev_finish = finish_here  # next leg departs after service here
                makespan = max(makespan, finish_here)

            idx = next_idx

        if seq:
            schedule[drone] = seq

    return schedule, makespan


# -------------------------- Example usage --------------------------
if __name__ == "__main__":
    # Paste your example directly:
    subtasks_with_drones = [
        {"name": "SubTask1", "skill": "RecordVideo",      "object": "Tower",   "service_time": 0.5, "drones": ["Drone1"]},
        {"name": "SubTask2", "skill": "RecordVideo",      "object": "House3",  "service_time": 0.5, "drones": ["Drone1"]},
        {"name": "SubTask3", "skill": "RecordVideo",      "object": "Base",    "service_time": 0.5, "drones": ["Drone1"]},
        {"name": "SubTask4", "skill": "CaptureRGBImage",  "object": "Tower",   "service_time": 3.0, "drones": ["Drone2", "Drone3"]},
        {"name": "SubTask5", "skill": "CaptureRGBImage",  "object": "House3",  "service_time": 3.0, "drones": ["Drone2", "Drone3"]},
        {"name": "SubTask6", "skill": "CaptureRGBImage",  "object": "Base",    "service_time": 3.0, "drones": ["Drone2", "Drone3"]},
    ]

    travel_times = {
        "drone_to_object": {
            "Drone1": {"Tower": 6.9, "House3": 5.5, "Base": 1.3},
            "Drone2": {"Tower": 3.7, "House3": 0.4, "Base": 4.4},
            "Drone3": {"Tower": 2.4, "House3": 2.5, "Base": 6.0},
        },
        "drone_object_to_object": {
            "Drone1": {
                "Tower":   {"Tower": 0.0, "House3": 5.6, "Base": 8.1},
                "House3":  {"Tower": 5.6, "House3": 0.0, "Base": 5.9},
                "Base":    {"Tower": 8.1, "House3": 5.9, "Base": 0.0},
            },
            "Drone2": {
                "Tower":   {"Tower": 0.0, "House3": 3.8, "Base": 5.4},
                "House3":  {"Tower": 3.8, "House3": 0.0, "Base": 3.9},
                "Base":    {"Tower": 5.4, "House3": 3.9, "Base": 0.0},
            },
            "Drone3": {
                "Tower":   {"Tower": 0.0, "House3": 4.5, "Base": 6.5},
                "House3":  {"Tower": 4.5, "House3": 0.0, "Base": 4.7},
                "Base":    {"Tower": 6.5, "House3": 4.7, "Base": 0.0},
            },
        },
    }

    schedule = solve_vrp(subtasks_with_drones, travel_times, time_limit_s=3)
    from pprint import pprint
    pprint(schedule, sort_dicts=False)
