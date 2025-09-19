# Maybe without flight times, maybe use some visual model

messages = [{"role": "system", "content": """
You decompose a natural-language drone task into a list of subtasks, then group subtasks that must use the same drone. Output only valid Python code.

INPUT FORMAT:
Task: <short description>

Allowed actions: [<ActionName>, ...]

Allowed objects: [
    {"name": <ObjectName>, "pos": (<x>, <y>)},
    ...
]

CONSTRAINTS:
1) If the Task says “with the same drone”, place those subtasks in the SAME group in execution order.
2) If there is no mention of same drone usage do not put them in the same group.
3) `groups[i].skills` must be the union of the `skill` fields of the subtasks in that group.
4) Preserve the natural execution order implied by the Task when ordering subtasks within a group.
5) For "deliver from X to Y":
   - Add a PickupPayload at X.
   - Add a ReleasePayload at Y.
   - ReleasePayload must be in the same group and always follow PickupPayload.

OUTPUT FORMAT:
# <SubTaskName>: <short description + same drone requirements>
# ...
             
subtasks = [
    {"name": <SubTaskName>, "skill": <SkillName>, "object": <ObjectName>, "pos": (<x>, <y>)},
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
Task: Take RGB image of RoofTop1 and thermal image of RoofTop2 with the same drone and take a thermal image of House1.

Allowed actions: ["CaptureRGBImage", "CaptureThermalImage", "PickupPayload", "ReleasePayload", "RecordVideo", "InspectStructure", "MeasureWind"]

Allowed objects: [
    {"name": "House1", "pos": (12, 87)},
    {"name": "RoofTop1", "pos": (45, 33)},
    {"name": "RoofTop2", "pos": (78, 62)},
    {"name": "SolarPanel1", "pos": (9, 14)},
    {"name": "SolarPanel2", "pos": (65, 90)}
]"""},
{"role": "assistant", "content": """
# SubTask1: Capture RGB image of RoofTop1
# SubTask2: Capture thermal image of RoofTop2 (Task mentions "with the same drone as" (SubTask1) -> same group as SubTask1)
# SubTask3: Capture thermal image of House1
 
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
Task: Inspect the structure of the Tower and pick up a payload from House2 and deliver it to House3.

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
# SubTask1: Inspect Tower
# SubTask2: Pick up payload at House2
# SubTask3: Release payload at House3 (release must use the same drone as pickup (SubTask2) -> same group as SubTask2)

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
Task: Take a thermal and an RGB image of Tower and measure the wind speed at House3 and at Base.

Allowed actions: ["CaptureRGBImage", "CaptureThermalImage", "MeasureWind", "InspectStructure"]

Allowed objects: [
    {"name": "Base", "pos": (48, 99)},
    {"name": "House3", "pos": (92, 44)},
    {"name": "Tower", "pos": (39, 2)},
]"""},
{"role":"assistant", "content": """
# SubTask1: Capture thermal image of Tower
# SubTask2: Capture RGB image of Tower
# SubTask3: Measure wind at House3
# SubTask4: Measure wind at Base
 
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
