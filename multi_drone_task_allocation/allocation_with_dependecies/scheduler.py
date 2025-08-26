messages = [{'role': 'system', "content": """"You build parallel "waves" from given list of subtasks.
             
Input:
subtasks = [
    {{'name': 'SubTask1', 'object': '<LocationName>', 'skill': '<SkillName>', 'depends_on': <'SubTaskX' or None>, 'same_drone_as': <'SubTaskX' or None>}},
    ...
]
             
Return ONLY:
schedule = {
  "same_drone_groups": [
    ["SubTaskA", "SubTaskB", ...],
    ...
  ],
  "waves": [
    {"name": "Wave1", "subtasks": ["SubTask...","SubTask..."]},
    {"name": "Wave2", "subtasks": ["SubTask..."]}
  ]
}

Rules:
1) Build same_drone_groups by collecting tasks connected via same_drone_as.
2) A subtask may appear only in a wave strictly AFTER all of its depends_on tasks’ waves.
3) In each wave, include at most ONE subtask from any same_drone_group (one drone cannot do two tasks at once).
4) Assign each task to the EARLIEST wave that satisfies all rules
5) Schedule as many tasks in the same wave as allowed by the rules.
6) Minimize the total number of waves.
"""},

# Example 1 ------------------------------------------------------------------------------------------
{'role':'user', 'content': '''
subtasks = [
    {'name': 'SubTask1', 'object': 'RoofTop1', 'skill': 'CaptureRGBImage', 'depends_on': None, 'same_drone_as': None},
    {'name': 'SubTask2', 'object': 'RoofTop2', 'skill': 'CaptureThermalImage', 'depends_on': None, 'same_drone_as': 'SubTask1'},
    {'name': 'SubTask3', 'object': 'House1', 'skill': 'CaptureThermalImage', 'depends_on': None, 'same_drone_as': None}
]'''},

{'role': 'assistant', 'content': '''
schedule = {
  "same_drone_groups": [
    { "name": "Group1", ["SubTask1", "SubTask2"] }
  ], 
  "waves": [
    {"name": "Wave1", "subtasks": ["SubTask1", "SubTask3"]},
    {"name": "Wave2", "subtasks": ["SubTask2"]}
  ]
}'''},

# Example 2 --------------------------------------------------------------------------------------------
{'role':'user', 'content':'''
subtasks = [
    {'name': 'SubTask1', 'object': 'House2', 'skill': 'PickupPayload', 'depends_on': None, 'same_drone_as': None},
    {'name': 'SubTask2', 'object': 'House3', 'skill': 'ReleasePayload', 'depends_on': 'SubTask1', 'same_drone_as': 'SubTask1'}
]'''},
{'role': 'assistant', 'content': '''
schedule = {
  "same_drone_groups": [
    { "name": "Group1", ["SubTask1", "SubTask2"] }
  ], 
  "waves": [
    {"name": "Wave1", "subtasks": ["SubTask1"]},
    {"name": "Wave2", "subtasks": ["SubTask2"]}
  ]
}'''},

# Example 3 --------------------------------------------------------------------------------------------
{'role':'user', 'content':'''
 subtasks = [
    {'name': 'SubTask1', 'object': 'House2', 'skill': 'CaptureRGBImage', 'depends_on': None, 'same_drone_as': None},
    {'name': 'SubTask2', 'object': 'House3', 'skill': 'CaptureRGBImage', 'depends_on': 'SubTask1', 'same_drone_as': 'SubTask1'},
    {'name': 'SubTask3', 'object': 'House1', 'skill': 'CaptureRGBImage', 'depends_on': 'SubTask2', 'same_drone_as': 'SubTask1'}
  ]'''},
{'role': 'assistant', 'content': '''
schedule = {
  "same_drone_groups": [
    { "name": "Group1", ["SubTask1", "SubTask2", "SubTask3"] }
  ], 
  "waves": [
    {"name": "Wave1", "subtasks": ["SubTask1"]},
    {"name": "Wave2", "subtasks": ["SubTask2"]},
    {"name": "Wave3", "subtasks": ["SubTask3"]}
  ]
}'''},

# Example 4 --------------------------------------------------------------------------------------------
{'role':'user', 'content':'''
subtasks = [
    {'name': 'SubTask1', 'object': 'Tower', 'skill': 'CaptureThermalImage', 'depends_on': None, 'same_drone_as': None},
    {'name': 'SubTask2', 'object': 'Tower', 'skill': 'CaptureRGBImage', 'depends_on': 'SubTask1', 'same_drone_as': None}
]'''},
{'role': 'assistant', 'content': '''
schedule = {
  "same_drone_groups": [],
  "waves": [
    {"name": "Wave1", "subtasks": ["SubTask1"]},
    {"name": "Wave2", "subtasks": ["SubTask2"]}
  ]
}'''},

# Example 5 --------------------------------------------------------------------------------------------
{'role':'user', 'content':'''
subtasks = [
    {'name': 'SubTask1', 'object': 'RoofTop1', 'skill': 'PickupPayload', 'depends_on': None, 'same_drone_as': None},
    {'name': 'SubTask2', 'object': 'House2', 'skill': 'ReleasePayload', 'depends_on': 'SubTask1', 'same_drone_as': 'SubTask1'},
    {'name': 'SubTask3', 'object': 'SolarPanel1', 'skill': 'CaptureThermalImage', 'depends_on': None, 'same_drone_as': None},
    {'name': 'SubTask4', 'object': 'SolarPanel2', 'skill': 'CaptureThermalImage', 'depends_on': None, 'same_drone_as': 'SubTask3'}
]'''},
{'role': 'assistant', 'content': '''
schedule = {
  "same_drone_groups": [
    { "name": "Group1", ["SubTask1", "SubTask2"] },
    { "name": "Group2", ["SubTask3", "SubTask4"] }
  ], 
  "waves": [
    {"name": "Wave1", "subtasks": ["SubTask1", "SubTask3"]},
    {"name": "Wave2", "subtasks": ["SubTask2", "SubTask4"]}
  ]
}'''},
]
