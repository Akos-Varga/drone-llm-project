# Input should be groups from decomposer and drones with skills

messages = [{"role": "system", "content": """"Your a
             
INPUT FORMAT:
groups = [
    {"name": <GroupName>, "subtasks": [<SubTaskName>, ...], "skills": [<SkillName>, ...]},
    ...
]
             
CONSTRAINTS:
1) Build same_drone_groups by collecting tasks connected via same_drone_as.
2) A subtask may appear only in a round strictly AFTER all of its depends_on tasks’ rounds.
3) In a round, include at most ONE subtask from any same_drone_group (one drone cannot do two tasks at once).
4) Assign each task to the EARLIEST round that satisfies all rules.
5) Schedule as many tasks in the same round as allowed by the rules.
6) Minimize the total number of rounds.
             
OUTPUT FORMAT:
schedule = {
  "same_drone_groups": [ {"name": "<GroupName>", "subtasks": ["<SubTaskName>", ...], "skills": ["<SkillName>", ...]}, ... ],
  "rounds": [ {"name": "<RoundName>", "subtasks": ["<SubTaskName>", ...]}, ... ],
  "subtasks": [ {"name": "<SubTaskName>", "skill": "<Skill>", "object": "<ObjectName>"}, ... ]
 }
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

"""},

# Example 2 --------------------------------------------------------------------------------------------
{"role":"user", "content":"""
drones = {
  "Drone1": {"skills": ["PickupPayload"], "pos": (42, 59), "speed": 14},
  "Drone2": {"skills": ["InspectStructure"], "pos": (21, 88), "speed": 17},
  "Drone3": {"skills": ["ReleasePayload"], "pos": (15, 77), "speed": 12},
  "Drone4": {"skills": ["MeasureWind"], "pos": (8, 25), "speed": 19},
  "Drone5": {"skills": ["PickupPayload", "ReleasePayload"], "pos": (67, 34), "speed": 11},
  "Drone6": {"skills": ["InspectStructure", "RecordVideo"], "pos": (90, 12), "speed": 16}
}

groups = [
  {"name": "Group1", "subtasks": ["SubTask1"], "skills": ["InspectStructure"]},
  {"name": "Group2", "subtasks": ["SubTask2", "SubTask3"], "skills": ["PickupPayload", "ReleasePayload"]}
]"""},
{"role": "assistant", "content": """

"""},

# Example 3 --------------------------------------------------------------------------------------------
{"role":"user", "content":"""
drones = {
  "Drone1": {"skills": ["CaptureThermalImage", "CaptureRGBImage"], "pos": (73, 12), "speed": 15},
  "Drone2": {"skills": ["MeasureWind"], "pos": (41, 85), "speed": 12},
  "Drone3": {"skills": ["CaptureRGBImage", "MeasureWind"], "pos": (96, 37), "speed": 18}
}
 
groups = [
  {"name": "Group1", "subtasks": ["SubTask1"], "skills": ["CaptureThermalImage"]},
  {"name": "Group2", "subtasks": ["SubTask2"], "skills": ["CaptureRGBImage"]},
  {"name": "Group3", "subtasks": ["SubTask3"], "skills": ["MeasureWind"]},
  {"name": "Group4", "subtasks": ["SubTask4"], "skills": ["MeasureWind"]},
]"""},
{"role": "assistant", "content": """

"""}
]
