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

if __name__ == "__main__":
    print(task_list[0]["solution"])