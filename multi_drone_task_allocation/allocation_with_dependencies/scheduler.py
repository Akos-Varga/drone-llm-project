messages = [{"role": "system", "content": """"You build parallel "rounds" from given list of subtasks outputting only Python code.
             
INPUT FORMAT:
subtasks = [
    {"name": "<SubTaskName>", "skill": "<SkillName>", "object": "<ObjectName>", "depends_on": "<SubTaskName>" or None, "same_drone_as": "<SubTaskName>" or None},
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
subtasks = [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "RoofTop1", "depends_on": None, "same_drone_as": None},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "RoofTop2", "depends_on": None, "same_drone_as": "SubTask1"},
    {"name": "SubTask3", "skill": "CaptureThermalImage", "object": "House1", "depends_on": None, "same_drone_as": None}
]"""},

{"role": "assistant", "content": """
schedule = {
  "same_drone_groups": [
    {"name": "Group1", "subtasks": ["SubTask1", "SubTask2"], "skills": ["CaptureRGBImage", "CaptureThermalImage"]}
  ], 
  "rounds": [
    {"name": "Round1", "subtasks": ["SubTask1", "SubTask3"]},
    {"name": "Round2", "subtasks": ["SubTask2"]}
  ],
  "subtasks": [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "RoofTop1"},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "RoofTop2"},
    {"name": "SubTask3", "skill": "CaptureThermalImage", "object": "House1"},
  ]
}"""},

# Example 2 --------------------------------------------------------------------------------------------
{"role":"user", "content":"""
subtasks = [
    {"name": "SubTask1", "skill": "PickupPayload", "object": "House2", "depends_on": None, "same_drone_as": None},
    {"name": "SubTask2", "skill": "ReleasePayload", "object": "House3", "depends_on": "SubTask1", "same_drone_as": "SubTask1"}
]"""},
{"role": "assistant", "content": """
schedule = {
  "same_drone_groups": [
    {"name": "Group1", ["SubTask1", "SubTask2"], "skills: ["PickupPayload", "ReleasePayload"]}
  ], 
  "rounds": [
    {"name": "Round1", "subtasks": ["SubTask1"]},
    {"name": "Round2", "subtasks": ["SubTask2"]}
  ],
  "subtasks": [
    { "name": "SubTask1", "skill": "PickupPayload", "object": "House2" },
    { "name": "SubTask2", "skill": "ReleasePayload", "object": "House3" }
  ]
}"""},

# Example 3 --------------------------------------------------------------------------------------------
{"role":"user", "content":"""
 subtasks = [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "House2", "depends_on": None, "same_drone_as": None},
    {"name": "SubTask2", "skill": "CaptureRGBImage", "object": "House3", "depends_on": "SubTask1", "same_drone_as": "SubTask1"},
    {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "House1", "depends_on": "SubTask2", "same_drone_as": "SubTask1"}
  ]"""},
{"role": "assistant", "content": """
schedule = {
  "same_drone_groups": [
    {"name": "Group1", ["SubTask1", "SubTask2", "SubTask3"], "skills": ["CaptureRGBImage"]}
  ], 
  "rounds": [
    {"name": "Round1", "subtasks": ["SubTask1"]},
    {"name": "Round2", "subtasks": ["SubTask2"]},
    {"name": "Round3", "subtasks": ["SubTask3"]}
  ],
  "subtasks": [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "House2"},
    {"name": "SubTask2", "skill": "CaptureRGBImage", "object": "House3"}, 
    {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "House1"}
  ]
}"""},

# Example 4 --------------------------------------------------------------------------------------------
{"role":"user", "content":"""
subtasks = [
    {"name": "SubTask1", "skill": "CaptureThermalImage", "object": "Tower", "depends_on": None, "same_drone_as": None},
    {"name": "SubTask2", "skill": "CaptureRGBImage", "object": "Tower", "depends_on": "SubTask1", "same_drone_as": None}
]"""},
{"role": "assistant", "content": """
schedule = {
  "same_drone_groups": [],
  "rounds": [
    {"name": "Round1", "subtasks": ["SubTask1"]},
    {"name": "Round2", "subtasks": ["SubTask2"]}
  ],
  "subtasks": [
    {"name": "SubTask1", "skill": "CaptureThermalImage", "object": "Tower"},
    {"name": "SubTask2", "skill": "CaptureRGBImage", "object": "Tower"}
  ]
}"""},

# Example 5 --------------------------------------------------------------------------------------------
{"role":"user", "content":"""
subtasks = [
    {"name": "SubTask1", "skill": "PickupPayload", "object": "RoofTop1", "depends_on": None, "same_drone_as": None},
    {"name": "SubTask2", "skill": "ReleasePayload", "object": "House2", "depends_on": "SubTask1", "same_drone_as": "SubTask1"},
    {"name": "SubTask3", "skill": "CaptureThermalImage", "object": "SolarPanel1", "depends_on": None, "same_drone_as": None},
    {"name": "SubTask4", "skill": "CaptureThermalImage", "object": "SolarPanel2", "depends_on": None, "same_drone_as": "SubTask3"}
]"""},
{"role": "assistant", "content": """
schedule = {
  "same_drone_groups": [
    {"name": "Group1", ["SubTask1", "SubTask2"], "skills": ["PickupPayload", "ReleasePayload"]},
    {"name": "Group2", ["SubTask3", "SubTask4"], "skills": ["CaptureThermalImage"]}
  ], 
  "rounds": [
    {"name": "Round1", "subtasks": ["SubTask1", "SubTask3"]},
    {"name": "Round2", "subtasks": ["SubTask2", "SubTask4"]}
  ],
  "subtasks": [
    {"name": "SubTask1", "skill": "PickupPayload", "object": "RoofTop1"},
    {"name": "SubTask2", "skill": "ReleasePayload", "object": "House2"},
    {"name": "SubTask3", "skill": "CaptureThermalImage", "object": "SolarPanel1"},
    {"name": "SubTask4", "skill": "CaptureThermalImage", "object": "SolarPanel2"}
  ]
}"""},
]