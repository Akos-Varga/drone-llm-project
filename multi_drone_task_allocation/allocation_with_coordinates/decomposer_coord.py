messages = [{"role": "system", "content": """
You decompose a natural-language drone task into a list of subtasks then group subtasks that use the same drone, outputting only Python code.

INPUT FORMAT:
Task: <short description>

Allowed actions: [<ActionName>, ...]

Allowed objects: [
    {"name": <ObjectName>, "pos": (<Xpos>, <Ypos>)},
    ...
]

CONSTRAINTS:
1) If two subtasks require the same drone, put them in one group in the order of their execution, and list the union of all skills required by subtasks in that group.
2) For “deliver from X to Y”:
   - Add a PickupPayload at X.
   - Add a ReleasePayload at Y.
   - ReleasePayload must be in the same group and always follow PickupPayload.

OUTPUT FORMAT:

subtasks = [
    {"name": <SubTaskName>, "skill": <SkillName>, "object": <ObjectName>, "pos": (<Xpos>, <Ypos>)},
    ...
]
groups = [
    {"name": <GroupName>, "subtasks": [<SubTaskName>, ...], "skills": [<SkillName>, ...]},
    ...
]
"""
},

# Example 1 ------------------------------------------------------------------------------------------
{"role": "user", "content": """
Task: Take RGB image of RoofTop1 and thermal image of RoofTop2 with the same drone. Take a thermal image of House1.

Allowed actions: ["CaptureRGBImage", "CaptureThermalImage", "PickupPayload", "ReleasePayload", "RecordVideo", "InspectStructure", "MeasureWind"]

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
    {"name": "SubTask3", "skill": "CaptureThermalImage", "object": "House1", "pos": (12, 87)}
 ]

groups = [
    {"name": "Group1", "subtasks": ["SubTask1", "SubTask2"], "skills": ["CaptureRGBImage", "CaptureThermalImage"]},
    {"name": "Group2", "subtasks": ["SubTask3"], "skills": ["CaptureThermalImage"]}
]"""},

# Example 2 --------------------------------------------------------------------------------------------
{"role": "user", "content": """
Task: Inspect the structure of the Tower. Pick up a payload from House2 and deliver it to House3.

Allowed actions: ["PickupPayload", "ReleasePayload", "RecordVideo", "InspectStructure", "MeasureWind"]

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
    {"name": "SubTask2", "skill": "PickupPayload", "object": "House2", "pos": (95, 64)},
    {"name": "SubTask3", "skill": "ReleasePayload", "object": "House3", "pos": (53, 49)}
    
]
 
groups = [
    {"name": "Group1", "subtasks": ["SubTask1"], "skills": ["InspectStructure"]},
    {"name": "Group2", "subtasks": ["SubTask2", "SubTask3"], "skills": ["PickupPayload", "ReleasePayload"]}
]"""},


# Example 3 --------------------------------------------------------------------------------------------
{"role":"user", "content": """
Task: Take a thermal image of Tower then take an RGB image of it. Measure the wind speed at House3 and at the Base.

Allowed actions: ["CaptureRGBImage", "CaptureThermalImage", "MeasureWind"]

Allowed objects: [
    {"name": "Base", "pos": (48, 99)},
    {"name": "House3", "pos": (92, 44)},
    {"name": "Tower", "pos": (39, 2)},
]"""},
{"role":"assistant", "content": """
subtasks = [
    {"name": "SubTask1", "skill": "CaptureThermalImage", "object": "Tower", "pos": (39, 2)},
    {"name": "SubTask2", "skill": "CaptureRGBImage", "object": "Tower", "pos": (39, 2)},
    {"name": "SubTask3", "skill": "MeasureWind", "object": "House3", "pos": (92, 44)},
    {"name": "SubTask4", "skill": "MeasureWind", "object": "Base", "pos": (48, 99)}
]

groups = [
    {"name": "Group1", "subtasks": ["SubTask1"], "skills": ["CaptureThermalImage"]},
    {"name": "Group2", "subtasks": ["SubTask2"], "skills": ["CaptureRGBImage"]},
    {"name": "Group3", "subtasks": ["SubTask3"], "skills": ["MeasureWind"]},
    {"name": "Group4", "subtasks": ["SubTask4"], "skills": ["MeasureWind"]},
]"""},
]
