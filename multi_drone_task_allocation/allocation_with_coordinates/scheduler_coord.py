messages = [
{"role": "system", "content": """
You are a task scheduler for a multi-drone system. Only output Python code.

INPUT FORMAT:
subtasks_with_drones = [
    {"name": <SubTaskName>, "skill": <SkillName>, "object": <ObjectName>, "pos": (<x>, <y>), "drones": [<DroneName>, ...]},
    ...
]
 
travel_times = {'drone_to_object':  {<DroneName>: {<ArrivalObjectName>: <TravelTime>, ...}, ...}, 'drone_object_to_object': {<DroneName>:{<DepartureObjectName>: {<ArrivalObjectName>: <TravelTime>, ...}, ...}, ...}}

CONSTRAINTS:
1) Each subtask name appears exactly once in schedule.
2) Tasks are instantaneous on arrival: no service time at the object.
3) Time semantics:
   a) startTime is the departure time from the drone’s current location; startTime >= 0.
   b) endTime is the arrival time at the task’s object.
   c) First task: endTime - startTime == travel_times['drone_to_object'][Drone][Object].
   d) For consecutive tasks from object A to B on the same drone: endTime(next) - startTime(next) == travel_times['drone_object_to_object'][Drone][A][B]
4) No overlapping intervals on the same drone.
5) Objective: minimize makespan (max over all endTime).

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
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "RoofTop1", "drones": ["Drone1", "Drone3", "Drone4"]},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "RoofTop2", "drones": ["Drone1", "Drone2", "Drone4"]},
    {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "House1", "drones": ["Drone1", "Drone3", "Drone4"]}
]

travel_times = {'drone_to_object': {'Drone1': {'RoofTop1': 3.5, 'RoofTop2': 4.1, 'House1': 1.1}, 'Drone3': {'RoofTop1': 4.1, 'House1': 8.0}, 'Drone4': {'RoofTop1': 1.4, 'RoofTop2': 2.3, 'House1': 1.9}, 'Drone2': {'RoofTop2': 3.1}}, 'drone_object_to_object': {'Drone1': {'RoofTop1': {'RoofTop1': 0.0, 'RoofTop2': 3.1, 'House1': 4.5}, 'RoofTop2': {'RoofTop1': 3.1, 'RoofTop2': 0.0, 'House1': 5.0}, 'House1': {'RoofTop1': 4.5, 'RoofTop2': 5.0, 'House1': 0.0}}, 'Drone3': {'RoofTop1': {'RoofTop1': 0.0, 'House1': 5.8}, 'House1': {'RoofTop1': 5.8, 'House1': 0.0}}, 'Drone4': {'RoofTop1': {'RoofTop1': 0.0, 'RoofTop2': 2.3, 'House1': 3.3}, 'RoofTop2': {'RoofTop1': 2.3, 'RoofTop2': 0.0, 'House1': 3.7}, 'House1': {'RoofTop1': 3.3, 'RoofTop2': 3.7, 'House1': 0.0}}, 'Drone2': {'RoofTop2': {'RoofTop2': 0.0}}}}"""
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
    {"name": "SubTask1", "skill": "InspectStructure", "object": "Tower", "drones": ["Drone2", "Drone4", "Drone5"]},
    {"name": "SubTask2", "skill": "MeasureWind", "object": "House2", "drones": ["Drone1", "Drone3"]},
    {"name": "SubTask3", "skill": "MeasureWind", "object": "House3", "drones": ["Drone1", "Drone3"]} 
]

travel_times = {'drone_to_object': {'Drone2': {'Tower': 4.5}, 'Drone4': {'Tower': 5.8}, 'Drone5': {'Tower': 5.2}, 'Drone1': {'House2': 3.8, 'House3': 1.1}, 'Drone3': {'House2': 5.0, 'House3': 2.7}}, 'drone_object_to_object': {'Drone2': {'Tower': {'Tower': 0.0}}, 'Drone4': {'Tower': {'Tower': 0.0}}, 'Drone5': {'Tower': {'Tower': 0.0}}, 'Drone1': {'House2': {'House2': 0.0, 'House3': 3.2}, 'House3': {'House2': 3.2, 'House3': 0.0}}, 'Drone3': {'House2': {'House2': 0.0, 'House3': 2.3}, 'House3': {'House2': 2.3, 'House3': 0.0}}}}"""
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
    {"name": "SubTask1", "skill": "RecordVideo", "object": "Tower", "drones": ["Drone1"]},
    {"name": "SubTask2", "skill": "RecordVideo", "object": "House3", "drones": ["Drone1"]},
    {"name": "SubTask3", "skill": "RecordVideo", "object": "Base", "drones": ["Drone1"]},
    {"name": "SubTask4", "skill": "CaptureRGBImage", "object": "Tower", "drones": ["Drone2", "Drone3"]},
    {"name": "SubTask5", "skill": "CaptureRGBImage", "object": "House3", "drones": ["Drone2", "Drone3"]},
    {"name": "SubTask6", "skill": "CaptureRGBImage", "object": "Base", "drones": ["Drone2", "Drone3"]}
]
 
travel_times = {'drone_to_object': {'Drone1': {'Tower': 6.9, 'House3': 5.5, 'Base': 1.3}, 'Drone2': {'Tower': 3.7, 'House3': 0.4, 'Base': 4.4}, 'Drone3': {'Tower': 2.4, 'House3': 2.5, 'Base': 6.0}}, 'drone_object_to_object': {'Drone1': {'Tower': {'Tower': 0.0, 'House3': 5.6, 'Base': 8.1}, 'House3': {'Tower': 5.6, 'House3': 0.0, 'Base': 5.9}, 'Base': {'Tower': 8.1, 'House3': 5.9, 'Base': 0.0}}, 'Drone2': {'Tower': {'Tower': 0.0, 'House3': 3.8, 'Base': 5.4}, 'House3': {'Tower': 3.8, 'House3': 0.0, 'Base': 3.9}, 'Base': {'Tower': 5.4, 'House3': 3.9, 'Base': 0.0}}, 'Drone3': {'Tower': {'Tower': 0.0, 'House3': 4.5, 'Base': 6.5}, 'House3': {'Tower': 4.5, 'House3': 0.0, 'Base': 4.7}, 'Base': {'Tower': 6.5, 'House3': 4.7, 'Base': 0.0}}}}"""
},
{"role": "assistant", "content": """
schedule = {
    "Drone1": [
        {"name": "SubTask1", "object": "Tower", "skill": "RecordVideo", "startTime": 0.0, "endTime": 6.9},
        {"name": "SubTask2", "object": "House3", "skill": "RecordVideo", "startTime": 6.9, "endTime": 12.5},
        {"name": "SubTask3", "object": "Base", "skill": "RecordVideo", "startTime": 12.5,"endTime": 18.4}
    ],
    "Drone2": [
        {"name": "SubTask5", "object": "House3", "skill": "CaptureRGBImage", "startTime": 0.0, "endTime": 0.4},
        {"name": "SubTask6", "object": "Base", "skill": "CaptureRGBImage", "startTime": 0.4, "endTime": 4.3}
    ],
    "Drone3": [
        {"name": "SubTask4", "object": "Tower", "skill": "CaptureRGBImage", "startTime": 0.0, "endTime": 2.4}
    ]
}
"""
},

# Example 4 --------------------------------------------------------------------------------------------
#{"role": "user", "content": """
#subtasks_with_drones = [
#    {"name": "SubTask1", "skill": "CaptureThermalImage", "object": "House2", "drones": ["Drone1", "Drone3", "Drone5"]},
#    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "House3", "drones": ["Drone1", "Drone3", "Drone5"]},
#    {"name": "SubTask3", "skill": "RecordVideo", "object": "SolarPanel1", "drones": ["Drone2", "Drone4"]},
#    {"name": "SubTask4", "skill": "RecordVideo", "object": "SolarPanel2", "drones": ["Drone2", "Drone4"]}
#]
# 
#travel_times = {'drone_to_object': {'Drone1': {'House2': 4.3, 'House3': 3.2}, 'Drone3': {'House2': 5.5, 'House3': 4.0}, 'Drone5': {'House2': 2.3, 'House3': 1.5}, 'Drone2': {'SolarPanel1': 3.1, 'SolarPanel2': 3.3}, 'Drone4': {'SolarPanel1': 4.8, 'SolarPanel2': 5.0}}, 'drone_object_to_object': {'Drone1': {'House2': {'House2': 0.0, 'House3': 1.3}, 'House3': {'House2': 1.3, 'House3': 0.0}}, 'Drone3': {'House2': {'House2': 0.0, 'House3': 1.5}, 'House3': {'House2': 1.5, 'House3': 0.0}}, 'Drone5': {'House2': {'House2': 0.0, 'House3': 2.0}, 'House3': {'House2': 2.0, 'House3': 0.0}}, 'Drone2': {'SolarPanel1': {'SolarPanel1': 0.0, 'SolarPanel2': 0.2}, 'SolarPanel2': {'SolarPanel1': 0.2, 'SolarPanel2': 0.0}}, 'Drone4': {'SolarPanel1': {'SolarPanel1': 0.0, 'SolarPanel2': 0.2}, 'SolarPanel2': {'SolarPanel1': 0.2, 'SolarPanel2': 0.0}}}}"""
#},
#{"role": "assistant", "content": """
## SubTask1 (House2): Drone1(start → House2)=4.3, Drone3(start → House2)=5.5, Drone5(start → House2)=2.3 → best: Drone5 (2.3)
## SubTask2 (House3): Drone1(start → House3)=3.2, Drone3(start → House3)=4.0, Drone5(House2 → House3)=2.3+2.0=4.3 → best: Drone1 (3.2)
## SubTask3 (SolarPanel1): Drone2(start → SolarPanel1)=3.1, Drone4(start → SolarPanel1)=4.8 → best: Drone2 (3.1)
## SubTask4 (SolarPanel2): Drone2(SolarPanel1 → SolarPanel2)=3.1+0.2=3.3, Drone4(start → SolarPanel2)=5.0 → best: Drone2 (3.3)
#
#schedule = {
#    "Drone1" : [
#        {"name": "SubTask2", "object": "House3", "skill": "CaptureThermalImage", "startTime": 0.0, "endTime": 3.2}
#    ],
#    "Drone2": [
#        {"name": "SubTask3", "object": "SolarPanel1", "skill": "RecordVideo", "startTime": 0.0, "endTime": 3.1},
#        {"name": "SubTask4", "object": "SolarPanel2", "skill": "RecordVideo", "startTime": 3.1, "endTime": 3.3}
#    ],
#    "Drone5": [
#        {"name": "SubTask1", "object": "House2", "skill": "CaptureThermalImage", "startTime": 0.0, "endTime": 2.3}
#    ]
#}
#"""
#},
#
## Example 5 --------------------------------------------------------------------------------------------
#{"role": "user", "content": """
#subtasks_with_drones = [
#    {"name": "SubTask1", "skill": "CaptureThermalImage", "object": "Tower", "drones": ["Drone1", "Drone3", "Drone5"]},
#    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "RoofTop1", "drones": ["Drone1", "Drone3", "Drone5"]}
#]
# 
#travel_times = {'drone_to_object': {'Drone1': {'Tower': 5.5, 'RoofTop1': 1.0}, 'Drone3': {'Tower': 4.1, 'RoofTop1': 3.3}, 'Drone5': {'Tower': 7.5, 'RoofTop1': 3.1}}, 'drone_object_to_object': {'Drone1': {'Tower': {'Tower': 0.0, 'RoofTop1': 5.6}, 'RoofTop1': {'Tower': 5.6, 'RoofTop1': 0.0}}, 'Drone3': {'Tower': {'Tower': 0.0, 'RoofTop1': 6.5}, 'RoofTop1': {'Tower': 6.5, 'RoofTop1': 0.0}}, 'Drone5': {'Tower': {'Tower': 0.0, 'RoofTop1': 8.4}, 'RoofTop1': {'Tower': 8.4, 'RoofTop1': 0.0}}}}"""
#},
#{"role": "assistant", "content": """
## SubTask1 (Tower): Drone1(start → Tower)=5.5, Drone3(start → Tower)=4.1, Drone5(start → Tower)=7.5 → best: Drone3 (4.1)
## SubTask2 (RoofTop1): Drone1(start → RoofTop1)=1.0, Drone3(Tower → RoofTop1)=4.1+6.5=10.6, Drone5(start → RoofTop1)=3.1 → best: Drone1 (1.0)
#schedule = {
#    "Drone1": [
#        {"name": "SubTask2", "object": "RoofTop1", "skill": "CaptureThermalImage", "startTime": 0.0, "endTime": 1.0}
#    ],
#    "Drone3": [
#        {"name": "SubTask1", "object": "Tower", "skill": "CaptureThermalImage", "startTime": 0.0, "endTime": 4.1}
#    ]
#}
#"""
#}
]
