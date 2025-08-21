messages = [{'role': 'system', "content": '''You are a task allocator for a multi-drone system.
             
INPUT includes:
- "same_drone_groups": [ ["<SubTaskName>", ...], ... ]
- "waves": [ { "name": "<WaveName>", "subtasks": ["<SubTaskName>", ...] }, ... ]
- "subtasks": [ {"name": "<SubTaskName>": { "skill": "<Skill>", "object": "<ObjectName>" }, ... }]

CONSTRAINTS
1) Assign each subtask only to a drone that has the required skill.
2) Subtasks within a "same_drone_group" MUST be assigned to the same drone, which must have all skills required by that group.
3) In the same wave, one drone MUST do only one subtask, so DO NOT use one drone more than once in one wave.
4) If multiple drones qualify, pick the one with the fewest skills.
             
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
             {"name": "SubTaskA", "drone": "DroneB},
             ...
             ]
'''},

# Example 1 ------------------------------------------------------------------------------------------
{'role':'user', 'content': '''{
  "same_drone_groups": [
    { "name": "Group1", "subtasks": ["SubTask1", "SubTask2"]}
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
 # Group1 contains SubTask1 and Subtask2, they both require CaptureRGBImage skill. This can be satisfied by Drone1, so Drone1 is allocated to SubTask1 and SubTask2.
 # SubTask3 requires CaptureThermalImage. This can be satisfied by Drone3. SubTask3 is in the same wave as SubTask1 but they use different drones so this is follows the rule. So Drone3 is allocated to SubTask3.

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
 # 
  # Group1 contains SubTask1 and Subtask2, they require PickupPayload and ReleasePayload skills. This can be satisfied by Drone6, so Drone1 is allocated to SubTask1 and SubTask2.
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
 # Group1 contains SubTask1, Subtask2 and SubTask3 they require CaptureRGBImage skill. This can be satisfied by Drone1, so Drone1 is allocated to SubTask1 and SubTask2.
 # SubTask3 requires CaptureThermalImage. This can be satisfied by Drone3. SubTask3 is in the same wave as SubTask1 but they use different drones so this is follows the rule. So Drone3 is allocated to SubTask3.

 allocated_subtasks = [
  {"name": "SubTask1", "drone": "Drone1"}, Same drone as SubTask2 and SubTask3 (same group)
  {"name": "SubTask2", "drone": "Drone1"}, # Same drone as SubTask1 and SubTask3 (same group)
  {"name": "SubTask3", "drone": "Drone1"} # Same drone as SubTask1 and SubTask2 (same group)
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
    ["SubTask1", "SubTask2"], # Needs to be allocated to the same drone
    ["SubTask3", "SubTask4"] # Needs to be allocated to the same drone
  ],
  "waves": [
    { "name": "Wave1", "subtasks": ["SubTask1", "SubTask3"] }, # Needs to be allocated to different drones
    { "name": "Wave2", "subtasks": ["SubTask2", "SubTask4"] } # Needs to be allocated to different drones
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
  {"name": "SubTask1", "drone": "Drone5"}, # Same drone as SubTask2 (same group), different drone than SubTask3 (same wave)
  {"name": "SubTask2", "drone": "Drone5"}, # Same drone as SubTask1 (same group), different drone than SubTask4 (same wave)
  {"name": "SubTask3", "drone": "Drone3"}, # Same drone as SubTask4 (same group), different drone than SubTask1 (same wave)
  {"name": "SubTask4", "drone": "Drone3"} # Same drone as SubTask3 (same group), different drone than SubTask2 (same wave)
]
'''}
]
