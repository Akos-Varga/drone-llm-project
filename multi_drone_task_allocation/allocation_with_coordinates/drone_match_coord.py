messages = [{"role": "system", "content": """
You match subtasks to capable drones. Only output Python code.
             
INPUT FORMAT:
drones = {
  <DroneName>: {"skills": [<SkillName>, ...], "pos": (<x>, <y>), "speed": <speed>},
  ...
}        

subtasks = [
    {"name": <SubTaskName>, "skill": <SkillName>, "object": <ObjectName>},
    ...
]
             
CONSTRAINTS:
1) A drone qualifies only if the skills listed for a drone include the skill required for the subtask.
2) Match by skills only, ignore position and speed.
3) If no drones qualify set the 'drones' list to [] for that subtask.

OUTPUT FORMAT:
subtasks_with_drones = [
    {"name": <SubTaskName>, "skill": <SkillName>, "object": <ObjectName>, "drones": [<DroneName>, ...]},
    ...
]"""
},

# Example 1 ------------------------------------------------------------------------------------------
{"role":"user", "content": """
drones = {
  "Drone1": {"skills": ["CaptureRGBImage", "CaptureThermalImage"], "pos": (23, 77), "speed": 14},
  "Drone2": {"skills": ["CaptureThermalImage"], "pos": (64, 12), "speed": 17},
  "Drone3": {"skills": ["CaptureRGBImage"], "pos": (89, 45), "speed": 11},
  "Drone4": {"skills": ["CaptureRGBImage", "CaptureThermalImage", "InspectStructure"], "pos": (35, 58), "speed": 19},
  "Drone5": {"skills": ["RecordVideo"], "pos": (10, 91), "speed": 13}
}

subtasks = [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "RoofTop1"},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "RoofTop2"},
    {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "House1"}
]"""},
{"role": "assistant", "content": """
subtasks_with_drones = [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "RoofTop1", "drones": ["Drone1", "Drone3", "Drone4"]},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "RoofTop2", "drones": ["Drone1", "Drone2", "Drone4"]},
    {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "House1", "drones": ["Drone1", "Drone3", "Drone4"]}
]
"""
},

# Example 2 --------------------------------------------------------------------------------------------
{"role":"user", "content":"""
drones = {
  "Drone1": {"skills": ["CaptureThermalImage", "MeasureWind"], "pos": (42, 59), "speed": 14},
  "Drone2": {"skills": ["InspectStructure"], "pos": (21, 88), "speed": 17},
  "Drone3": {"skills": ["MeasureWind"], "pos": (8, 25), "speed": 19},
  "Drone4": {"skills": ["InspectStructure"], "pos": (67, 34), "speed": 11},
  "Drone5": {"skills": ["InspectStructure", "RecordVideo"], "pos": (90, 12), "speed": 16}
}

subtasks = [
    {"name": "SubTask1", "skill": "InspectStructure", "object": "Tower"},
    {"name": "SubTask2", "skill": "MeasureWind", "object": "House2"},
    {"name": "SubTask3", "skill": "MeasureWind", "object": "House3"} 
]"""
},
{"role": "assistant", "content": """
subtasks_with_drones = [
    {"name": "SubTask1", "skill": "InspectStructure", "object": "Tower", "drones": ["Drone2", "Drone4", "Drone5"]},
    {"name": "SubTask2", "skill": "MeasureWind", "object": "House2", "drones": ["Drone1", "Drone3"]},
    {"name": "SubTask3", "skill": "MeasureWind", "object": "House3", "drones": ["Drone1", "Drone3"]} 
]"""
},

# Example 3 --------------------------------------------------------------------------------------------
{"role":"user", "content":"""
drones = {
  "Drone1": {"skills": ["MeasureWind", "RecordVideo"], "pos": (41, 85), "speed": 12},
  "Drone2": {"skills": ["CaptureRGBImage", "MeasureWind"], "pos": (96, 37), "speed": 18},
  "Drone3": {"skills": ["CaptureThermalImage", "InspectStructure", "CaptureRGBImage"], "pos": (73, 12), "speed": 15}
}
 
subtasks = [
    {"name": "SubTask1", "skill": "RecordVideo", "object": "Tower"},
    {"name": "SubTask2", "skill": "RecordVideo", "object": "House3"},
    {"name": "SubTask3", "skill": "RecordVideo", "object": "Base"},
    {"name": "SubTask4", "skill": "CaptureRGBImage", "object": "Tower"},
    {"name": "SubTask5", "skill": "CaptureRGBImage", "object": "House3"},
    {"name": "SubTask6", "skill": "CaptureRGBImage", "object": "Base"}
]"""
},
{"role": "assistant", "content": """
subtasks_with_drones = [
    {"name": "SubTask1", "skill": "RecordVideo", "object": "Tower", "drones": ["Drone1"]},
    {"name": "SubTask2", "skill": "RecordVideo", "object": "House3", "drones": ["Drone1"]},
    {"name": "SubTask3", "skill": "RecordVideo", "object": "Base", "drones": ["Drone1"]},
    {"name": "SubTask4", "skill": "CaptureRGBImage", "object": "Tower", "drones": ["Drone2", "Drone3"]},
    {"name": "SubTask5", "skill": "CaptureRGBImage", "object": "House3", "drones": ["Drone2", "Drone3"]},
    {"name": "SubTask6", "skill": "CaptureRGBImage", "object": "Base", "drones": ["Drone2", "Drone3"]}
]
"""
}
]
