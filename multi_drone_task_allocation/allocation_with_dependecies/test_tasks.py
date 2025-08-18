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

tasks_scheduled = ("Task1",)
