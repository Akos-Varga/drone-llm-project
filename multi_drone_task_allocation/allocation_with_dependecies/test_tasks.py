tasks =[("Task1", "Pick up a payload from SolarPanel1 and deliver it to Base."),
        ("Task2", "Take a thermal image of House2, after that with the same drone take an RGB image of House3."),
        ("Task3", "Deliver a package from House1 to Tower. Take RGB images of SolarPanel1 and SolarPanel2 with the same drone."),
        ("Task4", "Take thermal image of both solar panels. Take thermal image of all houses with the same drone."),
        ("Task5", "Take RGB image of the Tower then with this drone deliver a payload from Tower to House2.")
        ]

tasks_decomposed = [("Task1", '''subtasks = [
    {'name': 'SubTask1', 'object': 'SolarPanel1', 'skill': 'PickupPayload', 'depends_on': None, 'same_drone_as': None},
    {'name': 'SubTask2', 'object': 'Base', 'skill': 'ReleasePayload', 'depends_on': 'SubTask1', 'same_drone_as': 'SubTask1'}
]'''),
        ("Task2",'''subtasks = [
    {'name': 'SubTask1', 'object': 'House2', 'skill': 'CaptureThermalImage', 'depends_on': None, 'same_drone_as': None},
    {'name': 'SubTask2', 'object': 'House3', 'skill': 'CaptureRGBImage', 'depends_on': 'SubTask1', 'same_drone_as': 'SubTask1'}
]'''),
        ("Task3",'''subtasks = [
    {'name': 'SubTask1', 'object': 'House1', 'skill': 'PickupPayload', 'depends_on': None, 'same_drone_as': None},
    {'name': 'SubTask2', 'object': 'Tower', 'skill': 'ReleasePayload', 'depends_on': 'SubTask1', 'same_drone_as': 'SubTask1'},
    {'name': 'SubTask3', 'object': 'SolarPanel1', 'skill': 'CaptureRGBImage', 'depends_on': None, 'same_drone_as': None},
    {'name': 'SubTask4', 'object': 'SolarPanel2', 'skill': 'CaptureRGBImage', 'depends_on': None, 'same_drone_as': 'SubTask3'}
]'''),
        ("Task4",'''subtasks = [
    {'name': 'SubTask1', 'object': 'SolarPanel1', 'skill': 'CaptureThermalImage', 'depends_on': None, 'same_drone_as': None},
    {'name': 'SubTask2', 'object': 'SolarPanel2', 'skill': 'CaptureThermalImage', 'depends_on': None, 'same_drone_as': None},
    {'name': 'SubTask3', 'object': 'House1', 'skill': 'CaptureThermalImage', 'depends_on': None, 'same_drone_as': None},
    {'name': 'SubTask4', 'object': 'House2', 'skill': 'CaptureThermalImage', 'depends_on': None, 'same_drone_as': 'SubTask3'},
    {'name': 'SubTask5', 'object': 'House3', 'skill': 'CaptureThermalImage', 'depends_on': None, 'same_drone_as': 'SubTask3'}
]'''),
        ("Task5",'''subtasks = [
    {'name': 'SubTask1', 'object': 'Tower', 'skill': 'CaptureRGBImage', 'depends_on': None, 'same_drone_as': None},
    {'name': 'SubTask2', 'object': 'Tower', 'skill': 'PickupPayload', 'depends_on': 'SubTask1', 'same_drone_as': 'SubTask1'},
    {'name': 'SubTask3', 'object': 'House2', 'skill': 'ReleasePayload', 'depends_on': 'SubTask2', 'same_drone_as': 'SubTask2'}
]''')
        ]

tasks_scheduled = [("Task1", '''{
  "same_drone_groups": [
    { "name": "Group1", "subtasks": ["SubTask1", "SubTask2"] }
  ],
  "waves": [
    { "name": "Wave1", "subtasks": ["SubTask1"] },
    { "name": "Wave2", "subtasks": ["SubTask2"] }
  ],
  "subtasks": [
    { "name": "SubTask1", "skill": "PickupPayload",  "object": "SolarPanel1" },
    { "name": "SubTask2", "skill": "ReleasePayload", "object": "Base" }
  ]
}
'''),
("Task2", '''{
  "same_drone_groups": [
    { "name": "Group1", "subtasks": ["SubTask1", "SubTask2"] }
  ],
  "waves": [
    { "name": "Wave1", "subtasks": ["SubTask1"] },
    { "name": "Wave2", "subtasks": ["SubTask2"] }
  ],
  "subtasks": [
    { "name": "SubTask1", "skill": "CaptureThermalImage", "object": "House2" },
    { "name": "SubTask2", "skill": "CaptureRGBImage",     "object": "House3" }
  ]
}
'''),
("Task3", '''{
  "same_drone_groups": [
    { "name": "Group1", "subtasks": ["SubTask1", "SubTask2"] },
    { "name": "Group2", "subtasks": ["SubTask3", "SubTask4"] }
  ],
  "waves": [
    { "name": "Wave1", "subtasks": ["SubTask1", "SubTask3"] },
    { "name": "Wave2", "subtasks": ["SubTask2", "SubTask4"] }
  ],
  "subtasks": [
    { "name": "SubTask1", "skill": "PickupPayload",   "object": "House1" },
    { "name": "SubTask2", "skill": "ReleasePayload",  "object": "Tower" },
    { "name": "SubTask3", "skill": "CaptureRGBImage", "object": "SolarPanel1" },
    { "name": "SubTask4", "skill": "CaptureRGBImage", "object": "SolarPanel2" }
  ]
}
'''),
("Task4", '''{
  "same_drone_groups": [
    { "name": "Group1", "subtasks": ["SubTask3", "SubTask4", "SubTask5"] }
  ],
  "waves": [
    { "name": "Wave1", "subtasks": ["SubTask1", "SubTask2", "SubTask3"] },
    { "name": "Wave2", "subtasks": ["SubTask4"] },
    { "name": "Wave3", "subtasks": ["SubTask5"] }
  ],
  "subtasks": [
    { "name": "SubTask1", "skill": "CaptureThermalImage", "object": "SolarPanel1" },
    { "name": "SubTask2", "skill": "CaptureThermalImage", "object": "SolarPanel2" },
    { "name": "SubTask3", "skill": "CaptureThermalImage", "object": "House1" },
    { "name": "SubTask4", "skill": "CaptureThermalImage", "object": "House2" },
    { "name": "SubTask5", "skill": "CaptureThermalImage", "object": "House3" }
  ]
}
'''),
("Task5", '''{
  "same_drone_groups": [
    { "name": "Group1", "subtasks": ["SubTask1", "SubTask2", "SubTask3"] }
  ],
  "waves": [
    { "name": "Wave1", "subtasks": ["SubTask1"] },
    { "name": "Wave2", "subtasks": ["SubTask2"] },
    { "name": "Wave3", "subtasks": ["SubTask3"] }
  ],
  "subtasks": [
    { "name": "SubTask1", "skill": "CaptureRGBImage", "object": "Tower" },
    { "name": "SubTask2", "skill": "PickupPayload",   "object": "Tower" },
    { "name": "SubTask3", "skill": "ReleasePayload",  "object": "House2" }
  ]
}
''')
]

tasks_allocated = [
    ("Task1", '''allocated_subtasks = [
    {"name": "SubTask1", "drone": "Drone5"},
    {"name": "SubTask2", "drone": "Drone5"}
  ]
}
'''),
    ("Task2", '''allocated_subtasks = [
    {"name": "SubTask1", "drone": "Drone7"},
    {"name": "SubTask2", "drone": "Drone7"}
  ]
}
'''),
    ("Task3", '''allocated_subtasks = [
    {"name": "SubTask1", "drone": "Drone5"},
    {"name": "SubTask2", "drone": "Drone5"},
    {"name": "SubTask3", "drone": "Drone1"},
    {"name": "SubTask4", "drone": "Drone1"}
  ]
}
'''),
    ("Task4", '''allocated_subtasks = [
    {"name": "SubTask1", "drone": "Drone4"},
    {"name": "SubTask2", "drone": "Drone6"},
    {"name": "SubTask3", "drone": "Drone3"},
    {"name": "SubTask4", "drone": "Drone3"},
    {"name": "SubTask5", "drone": "Drone3"}
  ]
}
'''),
    ("Task5", '''allocated_subtasks = [
    {"name": "SubTask1", "drone": "Drone5"},
    {"name": "SubTask2", "drone": "Drone5"},
    {"name": "SubTask3", "drone": "Drone5"},''')
]
