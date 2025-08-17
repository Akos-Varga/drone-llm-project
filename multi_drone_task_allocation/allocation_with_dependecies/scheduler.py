messages = [{'role': 'system', "content": """"You build parallel "waves" from given list of subtasks.
             
Input:
subtasks = [
    {{'name': 'SubTask1', 'object': '<LocationName>', 'skill': '<SkillName>', 'depends_on': <'SubTaskX' or None>, 'same_drone_as': <'SubTaskX' or None>}},
    ...
]
             
Return only JSON of the form:
schedule = {
  "waves": [
    {"name": "Wave1", "subtasks": ["SubTask...","SubTask..."]},
    {"name": "Wave2", "subtasks": ["SubTask..."]}
  ],
  "same_drone_groups": [
    ["SubTaskA", "SubTaskB", ...],
    ...
  ]
}

Rules:
1) A task can be scheduled only after all its depends_on tasks' waves.
2) For any same_drone group without order, tasks must be in different waves with the group's anchor first.
3) Minimize number of waves.
"""},

# Example 1 ------------------------------------------------------------------------------------------
{'role':'user', 'content': '''
subtasks = [
    {'name': 'SubTask1', 'object': 'RoofTop1', 'skill': 'CaptureRGBImage', 'depends_on': None, 'same_drone_as': None},
    {'name': 'SubTask2', 'object': 'RoofTop2', 'skill': 'CaptureRGBImage', 'depends_on': None, 'same_drone_as': 'SubTask1'},
    {'name': 'SubTask3', 'object': 'House1', 'skill': 'CaptureThermalImage', 'depends_on': None, 'same_drone_as': None}
]'''},

{'role': 'assistant', 'content': '''
schedule = {
  "waves": [
    {"name": "Wave1", "subtasks": ["SubTask1", "SubTask3"]},
    {"name": "Wave2", "subtasks": ["SubTask2"]}
  ],
  "same_drone_groups": [
    ["SubTask1", "SubTask2"]
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
  "waves": [
    {"name": "Wave1", "subtasks": ["SubTask1"]},
    {"name": "Wave2", "subtasks": ["SubTask2"]}
  ],
  "same_drone_groups": [
    ["SubTask1", "SubTask2"]
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
  "waves": [
    {"name": "Wave1", "subtasks": ["SubTask1"]},
    {"name": "Wave2", "subtasks": ["SubTask2"]},
    {"name": "Wave3", "subtasks": ["SubTask3"]}
  ],
  "same_drone_groups": [
    ["SubTask1", "SubTask2", "SubTask3"]
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
  "waves": [
    {"name": "Wave1", "subtasks": ["SubTask1"]},
    {"name": "Wave2", "subtasks": ["SubTask2"]}
  ],
  "same_drone_groups": []
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
  "waves": [
    {"name": "Wave1", "subtasks": ["SubTask1", "SubTask3"]},
    {"name": "Wave2", "subtasks": ["SubTask2", "SubTask4"]}
  ],
  "same_drone_groups": [
    ["SubTask1", "SubTask2"],
    ["SubTask3", "SubTask4"]
  ]
}'''},
]
