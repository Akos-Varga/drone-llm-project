messages = [
{"role": "system", "content": """
You are a task allocator and scheduler for a multi-drone system. You output only Python code.

INPUT FORMAT:
subtasks_with_drones = [
    {"name": <SubTaskName>, "skill": <SkillName>, "object": <ObjectName>, "pos": (<x>, <y>), "drones": [<DroneName>, ...]},
    ...
]
 
travel_time = {'drone_to_object':  {<DroneName>: {<ArrivalObjectName>: <TravelTime>, ...}, ...}, 'drone_object_to_object': {<DroneName>:{<DepartureObjectName>: {<ArrivalObjectName>: <TravelTime>, ...}, ...}, ...}}

CONSTRAINTS:
1) Assign each subtask exactly once, and only to a drone listed in its "drones" field.
2) Each scheduled entry must use the subtask’s exact "skill" and "object".
3) Tasks are instantaneous on arrival: no service time at the object.
4) Time semantics:
   a) startTime is the departure time from the drone’s current location; startTime >= 0 (0 by default; waiting allowed).
   b) endTime is the arrival time at the task’s object.
   c) First leg equality: endTime - startTime == travel_time['drone_to_object'][Drone][Object].
   d) For consecutive tasks from object A to B on the same drone:
      - startTime(next) >= endTime(prev)
      - endTime(next) - startTime(next) == travel_time['drone_object_to_object'][Drone][A][B]
   e) Use the provided self-travel time when A == B (often 0.0).
5) Capacity: no overlapping intervals on the same drone. Intervals are half-open [startTime, endTime).
6) Objective: minimize makespan (max over all endTime).

OUTPUT FORMAT:
schedule = {
  <DroneName>: [
    {"name": <SubTaskName>, "object": <ObjectName>, "skill": <SkillName>, "startTime": <startTime>, "endTime": <endTime>},
    ...
 ],
 ...
}
"""},

# Example 1 ------------------------------------------------------------------------------------------
{"role": "user", "content": """
subtasks_with_drones = [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "RoofTop1", "pos": (45, 33), "drones": ["Drone1", "Drone3", "Drone4"]},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "RoofTop2", "pos": (78, 62), "drones": ["Drone1", "Drone2", "Drone4"]},
    {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "House1", "pos": (12, 87), "drones": ["Drone1", "Drone3", "Drone4"]}
]

travel_time = {'drone_to_object': {'Drone1': {'RoofTop1': 3.5, 'RoofTop2': 4.1, 'House1': 1.1}, 'Drone3': {'RoofTop1': 4.1, 'House1': 8.0}, 'Drone4': {'RoofTop1': 1.4, 'RoofTop2': 2.3, 'House1': 1.9}, 'Drone2': {'RoofTop2': 3.1}}, 'drone_object_to_object': {'Drone1': {'RoofTop1': {'RoofTop1': 0.0, 'RoofTop2': 3.1, 'House1': 4.5}, 'RoofTop2': {'RoofTop1': 3.1, 'RoofTop2': 0.0, 'House1': 5.0}, 'House1': {'RoofTop1': 4.5, 'RoofTop2': 5.0, 'House1': 0.0}}, 'Drone3': {'RoofTop1': {'RoofTop1': 0.0, 'House1': 5.8}, 'House1': {'RoofTop1': 5.8, 'House1': 0.0}}, 'Drone4': {'RoofTop1': {'RoofTop1': 0.0, 'RoofTop2': 2.3, 'House1': 3.3}, 'RoofTop2': {'RoofTop1': 2.3, 'RoofTop2': 0.0, 'House1': 3.7}, 'House1': {'RoofTop1': 3.3, 'RoofTop2': 3.7, 'House1': 0.0}}, 'Drone2': {'RoofTop2': {'RoofTop2': 0.0}}}}"""
},
{"role": "assistant", "content": """
schedule = {
    "Drone1": [
        {"name": "SubTask3", "object": "House1", "skill": "CaptureRGBImage", "startTime": 0, "endTime": 1.1}
    ],
    "Drone2": [
        {"name": "SubTask2", "object": "RoofTop2", "skill": "CaptureThermalImage", "startTime": 0, "endTime": 3.1}
    ],
    "Drone4": [
        {"name": "SubTask1", "object": "RoofTop1", "skill": "CaptureRGBImage", "startTime": 0, "endTime": 1.4}
    ]
}"""
},

# Example 2 --------------------------------------------------------------------------------------------
{"role": "user", "content": """
subtasks_with_drones = [
    {"name": "SubTask1", "skill": "InspectStructure", "object": "Tower", "pos": (7, 13), "drones": ["Drone2", "Drone4", "Drone5"]},
    {"name": "SubTask2", "skill": "MeasureWind", "object": "House2", "pos": (95, 64), "drones": ["Drone1", "Drone3"]},
    {"name": "SubTask3", "skill": "MeasureWind", "object": "House3", "pos": (53, 49), "drones": ["Drone1", "Drone3"]} 
]

travel_time = {'drone_to_object': {'Drone2': {'Tower': 4.5}, 'Drone4': {'Tower': 5.8}, 'Drone5': {'Tower': 5.2}, 'Drone1': {'House2': 3.8, 'House3': 1.1}, 'Drone3': {'House2': 5.0, 'House3': 2.7}}, 'drone_object_to_object': {'Drone2': {'Tower': {'Tower': 0.0}}, 'Drone4': {'Tower': {'Tower': 0.0}}, 'Drone5': {'Tower': {'Tower': 0.0}}, 'Drone1': {'House2': {'House2': 0.0, 'House3': 3.2}, 'House3': {'House2': 3.2, 'House3': 0.0}}, 'Drone3': {'House2': {'House2': 0.0, 'House3': 2.3}, 'House3': {'House2': 2.3, 'House3': 0.0}}}}"""
},
{"role": "assistant","content": """
schedule = {
    "Drone1": [
        {"name": "SubTask2", "object": "House2", "skill": "MeasureWind", "startTime": 0, "endTime": 3.8}
    ],
    "Drone2": [
        {"name": "SubTask1", "object": "Tower", "skill": "InspectStructure", "startTime": 0, "endTime": 4.5}
    ],
    "Drone3": [
        {"name": "SubTask3", "object": "House3", "skill": "MeasureWind", "startTime": 0, "endTime": 2.7}
    ]
}"""
},

# Example 3 --------------------------------------------------------------------------------------------
{"role": "user", "content": """
subtasks_with_drones = [
    {"name": "SubTask1", "skill": "RecordVideo", "object": "Tower", "pos": (39, 2), "drones": ["Drone1"]},
    {"name": "SubTask2", "skill": "RecordVideo", "object": "House3", "pos": (92, 44), "drones": ["Drone1"]},
    {"name": "SubTask3", "skill": "RecordVideo", "object": "Base", "pos": (48, 99), "drones": ["Drone1"]},
    {"name": "SubTask4", "skill": "CaptureRGBImage", "object": "Tower", "pos": (39, 2), "drones": ["Drone2", "Drone3"]},
    {"name": "SubTask5", "skill": "CaptureRGBImage", "object": "House3", "pos": (92, 44), "drones": ["Drone2", "Drone3"]},
    {"name": "SubTask6", "skill": "CaptureRGBImage", "object": "Base", "pos": (48, 99), "drones": ["Drone2", "Drone3"]}
]
 
travel_time = {'drone_to_object': {'Drone1': {'Tower': 6.9, 'House3': 5.5, 'Base': 1.3}, 'Drone2': {'Tower': 3.7, 'House3': 0.4, 'Base': 4.4}, 'Drone3': {'Tower': 2.4, 'House3': 2.5, 'Base': 6.0}}, 'drone_object_to_object': {'Drone1': {'Tower': {'Tower': 0.0, 'House3': 5.6, 'Base': 8.1}, 'House3': {'Tower': 5.6, 'House3': 0.0, 'Base': 5.9}, 'Base': {'Tower': 8.1, 'House3': 5.9, 'Base': 0.0}}, 'Drone2': {'Tower': {'Tower': 0.0, 'House3': 3.8, 'Base': 5.4}, 'House3': {'Tower': 3.8, 'House3': 0.0, 'Base': 3.9}, 'Base': {'Tower': 5.4, 'House3': 3.9, 'Base': 0.0}}, 'Drone3': {'Tower': {'Tower': 0.0, 'House3': 4.5, 'Base': 6.5}, 'House3': {'Tower': 4.5, 'House3': 0.0, 'Base': 4.7}, 'Base': {'Tower': 6.5, 'House3': 4.7, 'Base': 0.0}}}}"""
},
{"role": "assistant", "content": """
schedule = {
    "Drone1": [
        {"name": "SubTask3", "object": "Base",   "skill": "RecordVideo", "startTime": 0.0,  "endTime": 1.3},
        {"name": "SubTask2", "object": "House3", "skill": "RecordVideo", "startTime": 1.3,  "endTime": 7.2},  # +5.9 from Base
        {"name": "SubTask1", "object": "Tower",  "skill": "RecordVideo", "startTime": 7.2,  "endTime": 12.8} # +5.6 from House3
    ],
    "Drone2": [
        {"name": "SubTask5", "object": "House3", "skill": "CaptureRGBImage", "startTime": 0.0, "endTime": 0.4},
        {"name": "SubTask6", "object": "Base",   "skill": "CaptureRGBImage", "startTime": 0.4, "endTime": 4.3} # +3.9 from House3
    ],
    "Drone3": [
        {"name": "SubTask4", "object": "Tower",  "skill": "CaptureRGBImage", "startTime": 0.0, "endTime": 2.4}
    ]
}"""
}
]
