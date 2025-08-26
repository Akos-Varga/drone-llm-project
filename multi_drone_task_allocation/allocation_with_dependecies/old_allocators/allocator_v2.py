messages = [{'role': 'system', "content": '''You are a task allocator for a multi-drone system.
             
INPUT includes:
- "same_drone_groups": [ { "name": "<GroupName>", "subtasks": ["<SubTaskName>", ...] }, ... ]
- "waves": [ { "name": "<WaveName>", "subtasks": ["<SubTaskName>", ...] }, ... ]
- "subtasks": [ {"name": "<SubTaskName>": { "skill": "<Skill>", "object": "<ObjectName>" }, ... ]

CONSTRAINTS
1) Assign each subtask only to a drone that has the required skill.
2) Subtasks within a "same_drone_group" MUST be assigned to the same drone, which must have all skills required by that group.
3) In the same wave, one drone MUST do only one subtask, so DO NOT use one drone more than once in one wave.
4) If multiple drones qualify, choose the one with the fewest skills. If several drones tie for the fewest skills, select the drone with the smallest ID.
             
Available drones and their skills:
{
  "Drone1": ["CaptureRGBImage"],
  "Drone2": ["CaptureRGBImage"],
  "Drone3": ["CaptureThermalImage"],
  "Drone4": ["CaptureThermalImage"],
  "Drone5": ["CaptureRGBImage", "PickupPayload", "ReleasePayload"],
  "Drone6": ["PickupPayload", "ReleasePayload"],
  "Drone7": ["CaptureRGBImage", "CaptureThermalImage"],
  "Drone8": ["CaptureRGBImage"]
}

OUTPUT SCHEMA:
allocated_subtasks = [
             {"name": "SubTaskA", "drone": "DroneB"},
             ...
             ]
'''},

# Example 1 ------------------------------------------------------------------------------------------
{'role':'user', 'content': '''{
  "same_drone_groups": [
    { "name": "Group1", "subtasks": ["SubTask1", "SubTask2"] }
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
 # Group1 contains SubTask1 and Subtask2, so these subtasks need to be executed by the same drone. They both require CaptureRGBImage skill. This can be satisfied by Drone1, so Drone1 is allocated to SubTask1 and SubTask2.
 # SubTask3 is in the same wave as SubTask1 so it needs to be allocated to a different drone than SubTask1 (Drone1). SubTask3 requires CaptureThermalImage skill. This can be satisfied by Drone3. So Drone3 is allocated to SubTask3.

 allocated_subtasks = [
  {"name": "SubTask1", "drone": "Drone1"},
  {"name": "SubTask2", "drone": "Drone1"},
  {"name": "SubTask3", "drone": "Drone3"}
]'''},

# Example 2 ------------------------------------------------------------------------------------------
{'role':'user', 'content': '''{
  "same_drone_groups": [
    { "name": "Group1", "subtasks": ["SubTask1", "SubTask2"]}
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
  # Group1 contains SubTask1 and Subtask2, so these subtasks need to be executed by the same drone. They require PickupPayload and ReleasePayload skills. This can be satisfied by Drone6, so Drone6 is allocated to SubTask1 and SubTask2.

  allocated_subtasks = [
  {"name": "SubTask1", "drone": "Drone6"},
  {"name": "SubTask2", "drone": "Drone6"}
]'''},

# Example 3 ------------------------------------------------------------------------------------------
{'role':'user', 'content': '''{
  "same_drone_groups": [
    { "name": "Group1", "subtasks": ["SubTask1", "SubTask2", "SubTask3"]}
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
 # Group1 contains SubTask1, Subtask2 and SubTask3, so these subtasks need to be executed by the same drone. They require CaptureRGBImage skill. This can be satisfied by Drone1, so Drone1 is allocated to SubTask1, SubTask2 and SubTask3.

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
 # There are no drone groups for this task.
 # SubTask1 requires CaptureThermalImage skill. This can be satisfied by Drone3.
 # SubTask2 requires CaptureRGBImage skill. This can be satisfied by Drone1.

 allocated_subtasks = [
  {"name": "SubTask1", "drone": "Drone3"},
  {"name": "SubTask2", "drone": "Drone1"}
]
'''},

# Example 5 ------------------------------------------------------------------------------------------
{'role':'user', 'content': '''{
  "same_drone_groups": [
    { "name": "Group1", "subtasks": ["SubTask1", "SubTask2"]},
    { "name": "Group2", "subtasks": ["SubTask3", "SubTask4"]}
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
 # Group1 contains SubTask1 and SubTask2 so these need to be executed by the same drone. They require PickupPayload and ReleasePayload skills. This can be satisfied by Drone6.
 # Group2 contains SubTask3 and SubTask4 so these need to be executed by the same drone. SubTask3 is in the same wave as SubTask1 and SubTask4 is in the same wave as SubTask2 so they needs to be allocated to a different drone than SubTask1 and SubTask2 (Drone6). They require CaptureThermalImage skill. This can be satisfied by Drone3.

 allocated_subtasks = [
  {"name": "SubTask1", "drone": "Drone6"},
  {"name": "SubTask2", "drone": "Drone6"},
  {"name": "SubTask3", "drone": "Drone3"},
  {"name": "SubTask4", "drone": "Drone3"}
]
'''}
]
