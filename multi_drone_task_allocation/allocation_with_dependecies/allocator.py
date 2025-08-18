messages = [{'role': 'system', "content": '''You are a task allocator for a multi-drone system.

Available drones and their skills:
{
  "Drone1": ["CaptureRGBImage"],
  "Drone2": ["CaptureRGBImage"],
  "Drone3": ["CaptureThermalImage"],
  "Drone4": ["CaptureThermalImage"],
  "Drone5": ["PickupPayload", "ReleasePayload"],
  "Drone6": ["CaptureThermalImage"],
  "Drone7": ["CaptureRGBImage", "CaptureThermalImage"],
  "Drone8": ["CaptureRGBImage"]
}

INPUT JSON includes:
- "same_drone_groups": [ ["<SubTaskName>", ...], ... ]
- "waves": [ { "name": "<WaveName>", "subtasks": ["<SubTaskName>", ...] }, ... ]
- "subtasks": [ {"name": "<SubTaskName>": { "skill": "<Skill>", "object": "<ObjectName>" }, ... }]

OUTPUT a SINGLE valid JSON object assigning subtasks to drones, following these rules:

HARD CONSTRAINTS
1) Skill match: Assign each subtask only to a drone that has the required skill.
2) Same-drone groups: Subtasks in the same group must use the same drone, which must have all skills required by that group.
3) Per-wave exclusivity: In the same wave, one drone may do only one subtask.
4) Determinism: If multiple drones qualify, pick the one with the fewest skills.

OUTPUT SCHEMA:
allocated_subtasks = [
             {"name": "SubTaskA", "drone": "DroneB},
             ...
             ]
'''},

# Example 1 ------------------------------------------------------------------------------------------
{'role':'user', 'content': '''{
  "same_drone_groups": [
    ["SubTask1", "SubTask2"]
  ],
  "waves": [
    { "name": "Wave1", "subtasks": ["SubTask1", "SubTask3"] },
    { "name": "Wave2", "subtasks": ["SubTask2"] }
  ],
  "subtasks": [
    { "name": "SubTask1", "skill": "CaptureRGBImage", "object": "RoofTop1" },
    { "name": "SubTask2", "skill": "CaptureRGBImage", "object": "RoofTop2" },
    { "name": "SubTask3", "skill": "CaptureThermalImage", "object": "House1" },
  ]
}'''},
{'role':'assistant', 'content': '''
 allocated_subtasks = [
  {"name": "SubTask1", "drone": "Drone1"},
  {"name": "SubTask2", "drone": "Drone1"},
  {"name": "SubTask3", "drone": "Drone3"},
]'''},

# Example 2 ------------------------------------------------------------------------------------------
{'role':'user', 'content': '''{
  "same_drone_groups": [
    ["SubTask1", "SubTask2"]
  ],
  "waves": [
    { "name": "Wave1", "subtasks": ["SubTask1"] },
    { "name": "Wave2", "subtasks": ["SubTask2"] }
  ],
  "subtasks": [
    { "name": "SubTask1", "skill": "PickupPayload",  "object": "House2" },
    { "name": "SubTask2", "skill": "ReleasePayload", "object": "House3" }
  ]
}
'''},
{'role':'assistant', 'content': '''
 allocated_subtasks = [
  {"name": "SubTask1", "drone": "Drone5"},
  {"name": "SubTask2", "drone": "Drone5"}
]'''},

# Example 3 ------------------------------------------------------------------------------------------
{'role':'user', 'content': '''{
  "same_drone_groups": [
    ["SubTask1", "SubTask2", "SubTask3"]
  ],
  "waves": [
    { "name": "Wave1", "subtasks": ["SubTask1"] },
    { "name": "Wave2", "subtasks": ["SubTask2"] },
    { "name": "Wave3", "subtasks": ["SubTask3"] }
  ],
  "subtasks": [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "House2" },
    {"name": "SubTask2", "skill": "CaptureRGBImage", "object": "House3" },
    {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "House1" }
  ]
}
'''},
{'role':'assistant', 'content': '''
 allocated_subtasks = [
  {"name": "SubTask1", "drone": "Drone1"},
  {"name": "SubTask2", "drone": "Drone1"},
  {"name": "SubTask3", "drone": "Drone1"}
]
'''},

# Example 4 ------------------------------------------------------------------------------------------
{'role':'user', 'content': '''{
  "same_drone_groups": [],
  "waves": [
    { "name": "Wave1", "subtasks": ["SubTask1"] },
    { "name": "Wave2", "subtasks": ["SubTask2"] }
  ],
  "subtasks": [
    {"name": "SubTask1", "skill": "CaptureThermalImage", "object": "Tower" },
    {"name": "SubTask2", "skill": "CaptureRGBImage",    "object": "Tower" }
  ]
}
'''},
{'role':'assistant', 'content': '''
 allocated_subtasks = [
  {"name": "SubTask1", "drone": "Drone3"},
  {"name": "SubTask2", "drone": "Drone1"}
]
'''},

# Example 5 ------------------------------------------------------------------------------------------
{'role':'user', 'content': '''{
  "same_drone_groups": [
    ["SubTask1", "SubTask2"],
    ["SubTask3", "SubTask4"]
  ],
  "waves": [
    { "name": "Wave1", "subtasks": ["SubTask1", "SubTask3"] },
    { "name": "Wave2", "subtasks": ["SubTask2", "SubTask4"] }
  ],
  "subtasks": [
    {"name": "SubTask1", "skill": "PickupPayload",        "object": "RoofTop1" },
    {"name": "SubTask2", "skill": "ReleasePayload",       "object": "House2" },
    {"name": "SubTask3", "skill": "CaptureThermalImage",  "object": "SolarPanel1" },
    {"name": "SubTask4", "skill": "CaptureThermalImage",  "object": "SolarPanel2" }
  ]
}
'''},
{'role':'assistant', 'content': '''
 allocated_subtasks = [
  {"name": "SubTask1", "drone": "Drone5"},
  {"name": "SubTask2", "drone": "Drone5"},
  {"name": "SubTask3", "drone": "Drone3"},
  {"name": "SubTask4", "drone": "Drone3"}
]
'''}
]
