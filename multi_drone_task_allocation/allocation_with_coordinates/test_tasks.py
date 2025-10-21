# "CaptureRGBImage", "CaptureThermalImage", "RecordVideo", "InspectStructure", "MeasureWind"
# House1-3, SolarPanel1-2, Tower, Base, Rooftop1-2

tasks ={
  "Task1": "Inpect House2, measure wind at Base and record video of Tower.",
  "Task2": "Take a thermal image of House2 and RGB image of House3.",
  "Task3": "Take RGB images of SolarPanel1 and SolarPanel2.",
  "Task4": "Take thermal image of all houses.",
  "Task5": "Take RGB image and inspect Rooftop2.",
  "Task6": "Take RGB and thermal image of both SolarPanel1 and SolarPanel2.",
  "Task7": "Take RGB image of Base, Record Video of House1 and House3 and measure wind at Rooftop2.",
  "Task8": "Take thermal image of SolarPanel2 and SolarPanel1.",
  "Task9": "Take thermal image of Tower and thermal image of Rooftop1.",
  "Task10": "Take thermal image of House2 and House3, record a video of all solarpanels."
  }

task_list = [
    {"id": "Task1", "task": "Inpect House2, measure wind at Base and record video of Tower.", "solution": [
    {"name": "SubTask1", "skill": "InspectStructure", "object": "House2"},
    {"name": "SubTask2", "skill": "MeasureWind", "object": "Base"},
    {"name": "SubTask3", "skill": "RecordVideo", "object": "Tower"}
]},
    {"id": "Task2", "task": "Take a thermal image of House2 and RGB image of House3.", "solution":  [
    {"name": "SubTask1", "skill": "CaptureThermalImage", "object": "House2"},
    {"name": "SubTask2", "skill": "CaptureRGBImage", "object": "House3"}
]},
    {"id": "Task3", "task": "Take RGB images of SolarPanel1 and SolarPanel2.", "solution": [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "SolarPanel1"},
    {"name": "SubTask2", "skill": "CaptureRGBImage", "object": "SolarPanel2"}
]},
    {"id": "Task4", "task": "Take thermal image of all houses.", "solution": [
    {"name": "SubTask1", "skill": "CaptureThermalImage", "object": "House1"},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "House2"},
    {"name": "SubTask3", "skill": "CaptureThermalImage", "object": "House3"},
]},
    {"id": "Task5", "task": "Take RGB image and inspect Rooftop2.", "solution": [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "RoofTop2"},
    {"name": "SubTask2", "skill": "InspectStructure", "object": "RoofTop2"}
]},
    {"id": "Task6", "task": "Take RGB and thermal image of both SolarPanel1 and SolarPanel2.", "solution": [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "SolarPanel1"},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "SolarPanel1"},
    {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "SolarPanel2"},
    {"name": "SubTask4", "skill": "CaptureThermalImage", "object": "SolarPanel2"}
]},
    {"id": "Task7", "task": "Take RGB image of Base, Record Video of House1 and House3 and measure wind at Rooftop2.", "solution": [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "Base"},
    {"name": "SubTask2", "skill": "RecordVideo", "object": "House1"},
    {"name": "SubTask3", "skill": "RecordVideo", "object": "House3"},
    {"name": "SubTask4", "skill": "MeasureWind", "object": "RoofTop2"}
]},
    {"id": "Task8", "task": "Take thermal image of SolarPanel2 and SolarPanel1.", "solution": [
    {"name": "SubTask1", "skill": "CaptureThermalImage", "object": "SolarPanel2"},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "SolarPanel1"}
]},
    {"id": "Task9", "task": "Take thermal image of Tower and thermal image of Rooftop1.", "solution": [
    {"name": "SubTask1", "skill": "CaptureThermalImage", "object": "Tower"},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "RoofTop1"}
]},
    {"id": "Task10", "task": "Take thermal image of House2 and House3, record a video of all solarpanels.", "solution": [
    {"name": "SubTask1", "skill": "CaptureThermalImage", "object": "House2"},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "House3"},
    {"name": "SubTask3", "skill": "RecordVideo", "object": "SolarPanel1"},
    {"name": "SubTask4", "skill": "RecordVideo", "object": "SolarPanel2"}
]}
]

task_list2 = [
    # === 1-subtask tasks ===
    {"id": "Task1", "task": "Inspect the Tower for structural integrity.", "solution": [
        {"name": "SubTask1", "skill": "InspectStructure", "object": "Tower"}
    ]},
    {"id": "Task2", "task": "Take an RGB image of the Base.", "solution": [
        {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "Base"}
    ]},
    {"id": "Task3", "task": "Acquire a thermal image of House3.", "solution": [
        {"name": "SubTask1", "skill": "CaptureThermalImage", "object": "House3"}
    ]},

    # === 2-subtask tasks ===
    {"id": "Task4", "task": "Inspect RoofTop1 and measure wind conditions at RoofTop2.", "solution": [
        {"name": "SubTask1", "skill": "InspectStructure", "object": "RoofTop1"},
        {"name": "SubTask2", "skill": "MeasureWind", "object": "RoofTop2"}
    ]},
    {"id": "Task5", "task": "Record a video of House2 and collect a thermal image of House1.", "solution": [
        {"name": "SubTask1", "skill": "RecordVideo", "object": "House2"},
        {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "House1"}
    ]},
    {"id": "Task6", "task": "Measure wind at SolarPanel2 and at the Base.", "solution": [
        {"name": "SubTask1", "skill": "MeasureWind", "object": "SolarPanel2"},
        {"name": "SubTask2", "skill": "MeasureWind", "object": "Base"}
    ]},

    # === 3-subtask tasks ===
    {"id": "Task7", "task": "Capture RGB images of all solar panels and inspect the Tower.", "solution": [
        {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "SolarPanel1"},
        {"name": "SubTask2", "skill": "CaptureRGBImage", "object": "SolarPanel2"},
        {"name": "SubTask3", "skill": "InspectStructure", "object": "Tower"}
    ]},
    {"id": "Task8", "task": "Take thermal images of House1 and House3, followed by an inspection of RoofTop2.", "solution": [
        {"name": "SubTask1", "skill": "CaptureThermalImage", "object": "House1"},
        {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "House3"},
        {"name": "SubTask3", "skill": "InspectStructure", "object": "RoofTop2"}
    ]},
    {"id": "Task9", "task": "Record a video of the Base and obtain both RGB and thermal images of RoofTop1.", "solution": [
        {"name": "SubTask1", "skill": "RecordVideo", "object": "Base"},
        {"name": "SubTask2", "skill": "CaptureRGBImage", "object": "RoofTop1"},
        {"name": "SubTask3", "skill": "CaptureThermalImage", "object": "RoofTop1"}
    ]},

    # === 4-subtask tasks ===
    {"id": "Task10", "task": "Collect RGB and thermal imagery for all solar panels.", "solution": [
        {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "SolarPanel1"},
        {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "SolarPanel1"},
        {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "SolarPanel2"},
        {"name": "SubTask4", "skill": "CaptureThermalImage", "object": "SolarPanel2"}
    ]},
    {"id": "Task11", "task": "Inspect House1 and House2, and measure wind levels at the Tower and Base.", "solution": [
        {"name": "SubTask1", "skill": "InspectStructure", "object": "House1"},
        {"name": "SubTask2", "skill": "InspectStructure", "object": "House2"},
        {"name": "SubTask3", "skill": "MeasureWind", "object": "Tower"},
        {"name": "SubTask4", "skill": "MeasureWind", "object": "Base"}
    ]},
    {"id": "Task12", "task": "Record videos of both rooftops and take RGB images of each.", "solution": [
        {"name": "SubTask1", "skill": "RecordVideo", "object": "RoofTop1"},
        {"name": "SubTask2", "skill": "RecordVideo", "object": "RoofTop2"},
        {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "RoofTop1"},
        {"name": "SubTask4", "skill": "CaptureRGBImage", "object": "RoofTop2"}
    ]},

    # === 5-subtask tasks ===
    {"id": "Task13", "task": "Take RGB images of House1 and House2, collect a thermal image of House2, inspect RoofTop1, and measure wind at the Tower.", "solution": [
        {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "House1"},
        {"name": "SubTask2", "skill": "CaptureRGBImage", "object": "House2"},
        {"name": "SubTask3", "skill": "CaptureThermalImage", "object": "House2"},
        {"name": "SubTask4", "skill": "InspectStructure", "object": "RoofTop1"},
        {"name": "SubTask5", "skill": "MeasureWind", "object": "Tower"}
    ]},
    {"id": "Task14", "task": "Record videos of all solar panels, capture an RGB and thermal image of House1, and inspect the Tower.", "solution": [
        {"name": "SubTask1", "skill": "RecordVideo", "object": "SolarPanel1"},
        {"name": "SubTask2", "skill": "RecordVideo", "object": "SolarPanel2"},
        {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "House1"},
        {"name": "SubTask4", "skill": "CaptureThermalImage", "object": "House1"},
        {"name": "SubTask5", "skill": "InspectStructure", "object": "Tower"}
    ]},
    {"id": "Task15", "task": "Take thermal images of both rooftops and the Tower, measure wind at the Base, and capture an RGB image of House3.", "solution": [
        {"name": "SubTask1", "skill": "CaptureThermalImage", "object": "RoofTop1"},
        {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "RoofTop2"},
        {"name": "SubTask3", "skill": "CaptureThermalImage", "object": "Tower"},
        {"name": "SubTask4", "skill": "MeasureWind", "object": "Base"},
        {"name": "SubTask5", "skill": "CaptureRGBImage", "object": "House3"}
    ]},

    # === 6-subtask tasks ===
    {"id": "Task16", "task": "For all houses, collect both RGB and thermal imagery.", "solution": [
        {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "House1"},
        {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "House1"},
        {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "House2"},
        {"name": "SubTask4", "skill": "CaptureThermalImage", "object": "House2"},
        {"name": "SubTask5", "skill": "CaptureRGBImage", "object": "House3"},
        {"name": "SubTask6", "skill": "CaptureThermalImage", "object": "House3"}
    ]},
    {"id": "Task17", "task": "Inspect both rooftops and all solar panels, and measure wind at the Tower and Base.", "solution": [
        {"name": "SubTask1", "skill": "InspectStructure", "object": "RoofTop1"},
        {"name": "SubTask2", "skill": "InspectStructure", "object": "RoofTop2"},
        {"name": "SubTask3", "skill": "InspectStructure", "object": "SolarPanel1"},
        {"name": "SubTask4", "skill": "InspectStructure", "object": "SolarPanel2"},
        {"name": "SubTask5", "skill": "MeasureWind", "object": "Tower"},
        {"name": "SubTask6", "skill": "MeasureWind", "object": "Base"}
    ]},
    {"id": "Task18", "task": "Record videos of all houses and collect thermal images of both rooftops and the Tower.", "solution": [
        {"name": "SubTask1", "skill": "RecordVideo", "object": "House1"},
        {"name": "SubTask2", "skill": "RecordVideo", "object": "House2"},
        {"name": "SubTask3", "skill": "RecordVideo", "object": "House3"},
        {"name": "SubTask4", "skill": "CaptureThermalImage", "object": "RoofTop1"},
        {"name": "SubTask5", "skill": "CaptureThermalImage", "object": "RoofTop2"},
        {"name": "SubTask6", "skill": "CaptureThermalImage", "object": "Tower"}
    ]}
]


if __name__ == "__main__":
    print(task_list2[0]["solution"])