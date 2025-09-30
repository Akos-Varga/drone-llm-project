messages = [{"role": "system", "content": """
You decompose a natural-language drone task into a list of subtasks. Output only valid Python code.

INPUT FORMAT:
task = <short description>

actions = [<ActionName>, ...]

objects = {
    <ObjectName>: (<x>, <y>),
    ...
}

OUTPUT FORMAT:
subtasks = [
    {"name": <SubTaskName>, "skill": <SkillName>, "object": <ObjectName>},
    ...
]"""
},

# Example 1 ------------------------------------------------------------------------------------------
{"role": "user", "content": """
task = "Take RGB image of RoofTop1, thermal image of RoofTop2 and RGB image of House1."

actions = ["CaptureRGBImage", "CaptureThermalImage", "RecordVideo", "InspectStructure", "MeasureWind"]

objects = {
    "House1": (12, 87),
    "RoofTop1": (45, 33),
    "RoofTop2": (78, 62),
    "SolarPanel1": (9, 14),
    "SolarPanel2": (65, 90)
}"""
},
{"role": "assistant", "content": """
subtasks = [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "RoofTop1"},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "RoofTop2"},
    {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "House1"}
]"""},

# Example 2 --------------------------------------------------------------------------------------------
{"role": "user", "content": """
task = "Inspect the structure of the Tower and measure the wind speed at House2 and House3."

actions = ["RecordVideo", "InspectStructure", "MeasureWind"]

objects = {
    "House2": (12, 87),
    "RoofTop1": (45, 33),
    "Tower": (78, 62),
    "House3": (9, 14),
    "SolarPanel2": (65, 90)
}
"""},
{"role": "assistant", "content": """
subtasks = [
    {"name": "SubTask1", "skill": "InspectStructure", "object": "Tower"},
    {"name": "SubTask2", "skill": "MeasureWind", "object": "House2"},
    {"name": "SubTask3", "skill": "MeasureWind", "object": "House3"} 
]"""},

# Example 3 --------------------------------------------------------------------------------------------
{"role":"user", "content": """
task = "Record video and take RGB image of Tower, House3 and Base."

actions = ["CaptureRGBImage", "CaptureThermalImage", "MeasureWind", "RecordVideo"]

objects = {
    "Base": (48, 99),
    "House3": (92, 44),
    "Tower": (39, 2)
}
"""},
{"role":"assistant", "content": """
subtasks = [
    {"name": "SubTask1", "skill": "RecordVideo", "object": "Tower"},
    {"name": "SubTask2", "skill": "RecordVideo", "object": "House3"},
    {"name": "SubTask3", "skill": "RecordVideo", "object": "Base"},
    {"name": "SubTask4", "skill": "CaptureRGBImage", "object": "Tower"},
    {"name": "SubTask5", "skill": "CaptureRGBImage", "object": "House3"},
    {"name": "SubTask6", "skill": "CaptureRGBImage", "object": "Base"}
]"""}
]
