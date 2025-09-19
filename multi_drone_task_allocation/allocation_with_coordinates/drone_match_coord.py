messages = [{"role": "system", "content": """
You match capable drones to a list of skills. Only output Python code.
             
INPUT FORMAT:
drones = {
  <DroneName>: {"skills": [<SkillName>, ...], "pos": (<x>, <y>), "speed": <speed>},
  ...
}        

groups = [
  {"name": <GroupName>, "subtasks": [<SubTaskName>, ...], "skills": [<SkillName>, ...]},
  ...
]
             
CONSTRAINTS:
1) A drone qualifies only if every skill required by the group is present in that drone’s skill list. Do not include drones that are missing even one required skill.
2) Match by skills only, ignore position and speed.
3) If no drones qualify use an empty list '[]'.
             
OUTPUT FORMAT:
groups_with_drones = [
  {"name": <GroupName>, "subtasks": [<SubTaskName>, ...], "skills": [<SkillName>, ...], "drones": [<DroneName>, ...]},
  ...
]
"""},

# Example 1 ------------------------------------------------------------------------------------------
{"role":"user", "content": """
drones = {
  "Drone1": {"skills": ["CaptureRGBImage", "CaptureThermalImage"], "pos": (23, 77), "speed": 14},
  "Drone2": {"skills": ["CaptureThermalImage"], "pos": (64, 12), "speed": 17},
  "Drone3": {"skills": ["CaptureRGBImage"], "pos": (89, 45), "speed": 11},
  "Drone4": {"skills": ["CaptureRGBImage", "CaptureThermalImage", "InspectStructure"], "pos": (35, 58), "speed": 19},
  "Drone5": {"skills": ["PickupPayload", "ReleasePayload"], "pos": (10, 91), "speed": 13}
}

groups = [
  {"name": "Group1", "subtasks": ["SubTask1", "SubTask2"], "skills": ["CaptureRGBImage", "CaptureThermalImage"]},
  {"name": "Group2", "subtasks": ["SubTask3"], "skills": ["CaptureThermalImage"]}
]"""},
{"role": "assistant", "content": """
groups_with_drones = [
  {"name": "Group1", "subtasks": ["SubTask1", "SubTask2"], "skills": ["CaptureRGBImage", "CaptureThermalImage"], "drones": ["Drone1", "Drone4"]},
  {"name": "Group2", "subtasks": ["SubTask3"], "skills": ["CaptureThermalImage"], "drones": ["Drone1", "Drone2", "Drone4"]}
]
"""},

# Example 2 --------------------------------------------------------------------------------------------
{"role":"user", "content":"""
drones = {
  "Drone1": {"skills": ["PickupPayload", "ReleasePayload"], "pos": (42, 59), "speed": 14},
  "Drone2": {"skills": ["InspectStructure"], "pos": (21, 88), "speed": 17},
  "Drone3": {"skills": ["MeasureWind"], "pos": (8, 25), "speed": 19},
  "Drone4": {"skills": ["PickupPayload", "ReleasePayload"], "pos": (67, 34), "speed": 11},
  "Drone5": {"skills": ["InspectStructure", "RecordVideo"], "pos": (90, 12), "speed": 16}
}

groups = [
  {"name": "Group1", "subtasks": ["SubTask1"], "skills": ["InspectStructure"]},
  {"name": "Group2", "subtasks": ["SubTask2", "SubTask3"], "skills": ["PickupPayload", "ReleasePayload"]}
]"""},
{"role": "assistant", "content": """
groups_with_drones = [
  {"name": "Group1", "subtasks": ["SubTask1"], "skills": ["InspectStructure"], "drones": ["Drone2", "Drone5"]},
  {"name": "Group2", "subtasks": ["SubTask2", "SubTask3"], "skills": ["PickupPayload", "ReleasePayload"], "drones": ["Drone1", "Drone4"]}
]
"""},

# Example 3 --------------------------------------------------------------------------------------------
{"role":"user", "content":"""
drones = {
  "Drone1": {"skills": ["MeasureWind"], "pos": (41, 85), "speed": 12},
  "Drone2": {"skills": ["CaptureRGBImage", "MeasureWind"], "pos": (96, 37), "speed": 18},
  "Drone3": {"skills": ["CaptureThermalImage", "InspectStructure", "CaptureRGBImage"], "pos": (73, 12), "speed": 15}
}
 
groups = [
  {"name": "Group1", "subtasks": ["SubTask1"], "skills": ["CaptureThermalImage"]},
  {"name": "Group2", "subtasks": ["SubTask2"], "skills": ["CaptureRGBImage"]},
  {"name": "Group3", "subtasks": ["SubTask3"], "skills": ["MeasureWind"]},
  {"name": "Group4", "subtasks": ["SubTask4"], "skills": ["MeasureWind"]}
]"""},
{"role": "assistant", "content": """
groups_with_drones = [
  {"name": "Group1", "subtasks": ["SubTask1"], "skills": ["CaptureThermalImage"], "drones": ["Drone3"]},
  {"name": "Group2", "subtasks": ["SubTask2"], "skills": ["CaptureRGBImage"], "drones": ["Drone2", "Drone3"]},
  {"name": "Group3", "subtasks": ["SubTask3"], "skills": ["MeasureWind"], "drones": ["Drone1", "Drone2"]},
  {"name": "Group4", "subtasks": ["SubTask4"], "skills": ["MeasureWind"], "drones": ["Drone1", "Drone2"]}
]
"""}
]
