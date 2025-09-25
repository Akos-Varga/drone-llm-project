messages = [{"role": "system", "content": """
You decompose a natural-language drone task into a list of subtasks. Output only valid Python code.

INPUT FORMAT:
Task: <short description>

Allowed actions: [<ActionName>, ...]

Allowed objects: [
    {"name": <ObjectName>, "pos": (<x>, <y>)},
    ...
]

OUTPUT FORMAT:
# <SubTaskName>: <short description>
# ...
             
subtasks = [
    {"name": <SubTaskName>, "skill": <SkillName>, "object": <ObjectName>, "pos": (<x>, <y>)},
    ...
]"""
},

# Example 1 ------------------------------------------------------------------------------------------
{"role": "user", "content": """
Task: Take RGB image of RoofTop1, thermal image of RoofTop2 and RGB image of House1.

Allowed actions: ["CaptureRGBImage", "CaptureThermalImage", "RecordVideo", "InspectStructure", "MeasureWind"]

Allowed objects: [
    {"name": "House1", "pos": (12, 87)},
    {"name": "RoofTop1", "pos": (45, 33)},
    {"name": "RoofTop2", "pos": (78, 62)},
    {"name": "SolarPanel1", "pos": (9, 14)},
    {"name": "SolarPanel2", "pos": (65, 90)}
]"""},
{"role": "assistant", "content": """
subtasks = [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "RoofTop1", "pos": (45, 33)},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "RoofTop2", "pos": (78, 62)},
    {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "House1", "pos": (12, 87)}
]"""},

# Example 2 --------------------------------------------------------------------------------------------
{"role": "user", "content": """
Task: Inspect the structure of the Tower and measure the wind speed at House2 and House3

Allowed actions: ["RecordVideo", "InspectStructure", "MeasureWind"]

Allowed objects: [
    {"name": "Base", "pos": (34, 72)},
    {"name": "RoofTop1", "pos": (61, 5)},
    {"name": "SolarPanel2", "pos": (44, 87)},
    {"name": "House1", "pos": (12, 28)},
    {"name": "House2", "pos": (95, 64)},
    {"name": "House3", "pos": (53, 49)},
    {"name": "Tower", "pos": (7, 13)}
]"""},
{"role": "assistant", "content": """
subtasks = [
    {"name": "SubTask1", "skill": "InspectStructure", "object": "Tower", "pos": (7, 13)},
    {"name": "SubTask2", "skill": "MeasureWind", "object": "House2", "pos": (95, 64)},
    {"name": "SubTask3", "skill": "MeasureWind", "object": "House3", "pos": (53, 49)} 
]"""},

# Example 3 --------------------------------------------------------------------------------------------
{"role":"user", "content": """
Task: Record video and take RGB image of Tower, House3 and Base.

Allowed actions: ["CaptureRGBImage", "CaptureThermalImage", "MeasureWind", "RecordVideo"]

Allowed objects: [
    {"name": "Base", "pos": (48, 99)},
    {"name": "House3", "pos": (92, 44)},
    {"name": "Tower", "pos": (39, 2)},
]"""},
{"role":"assistant", "content": """
subtasks = [
    {"name": "SubTask1", "skill": "RecordVideo", "object": "Tower", "pos": (39, 2)},
    {"name": "SubTask2", "skill": "RecordVideo", "object": "House3", "pos": (92, 44)},
    {"name": "SubTask3", "skill": "RecordVideo", "object": "Base", "pos": (48, 99)},
    {"name": "SubTask4", "skill": "CaptureRGBImage", "object": "Tower", "pos": (39, 2)},
    {"name": "SubTask5", "skill": "CaptureRGBImage", "object": "House3", "pos": (92, 44)},
    {"name": "SubTask6", "skill": "CaptureRGBImage", "object": "Base", "pos": (48, 99)}
]"""},
]
