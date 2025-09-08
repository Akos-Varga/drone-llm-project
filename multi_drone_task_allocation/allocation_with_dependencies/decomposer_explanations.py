ACTIONS = ["CaptureRGBImage", "CaptureThermalImage", "PickupPayload", "ReleasePayload"]
OBJECTS = ["Base", "RoofTop1", "RoofTop2", "SolarPanel1", "SolarPanel2", "House1", "House2", "House3", "Tower"]

messages = [{"role": "system", "content": f"""You decompose a natural-language drone task into a list of subtasks outputting only Python code.
  
ALLOWED SKILLS: {", ".join(ACTIONS)}
ALLOWED OBJECTS: {", ".join(OBJECTS)}

CONSTRAINTS:
1) Add a temporal dependency whenever the text specifies an order (e.g., "after", "then", "before").
2) Add a same-drone dependency whenever the text specifies that subtasks use the same drone.
3) Use `depends_on` for temporal dependencies.
4) Use `same_drone_as` for same-drone dependencies.
5) For “deliver from X to Y”:
   - Add a PickupPayload at X.
   - Add a ReleasePayload at Y.
   - ReleasePayload depends_on PickupPayload and same_drone_as PickupPayload.

OUTPUT FORMAT:
# <SubTaskName>: <short description>
# ...

# TEMPORAL DEPENDENCIES (THEN/AFTER/BEFORE)
# <SubTaskB> depends_on <SubTaskA>  [reason: "<brief cue from text>"]
# ...

# SAME-DRONE DEPENDENCIES
# <SubTaskX> same_drone_as <SubTaskY>  [reason: "<brief cue from text>"]
# ...

subtasks = [
    {{"name": "<SubTaskName>", "skill": "<SkillName>", "object": "<ObjectName>", "depends_on": "<SubTaskName>" or None, "same_drone_as": "<SubTaskName>" or None}},
    ...
]
"""
},

# Example 1 ------------------------------------------------------------------------------------------
{"role": "user", "content": "Take RGB image of RoofTop1 and thermal image of RoofTop2 with the same drone. Take thermal image of House1."},
{"role": "assistant", "content": """# SubTask1: Take RGB image of RoofTop1
# SubTask2: Take thermal image of RoofTop2
# SubTask3: Take thermal image of House1

# TEMPORAL DEPENDENCIES (THEN/AFTER/BEFORE)
# (none)

# SAME-DRONE DEPENDENCIES
# SubTask2 same_drone_as SubTask1  [reason: "with the same drone"]

subtasks = [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "RoofTop1", "depends_on": None, "same_drone_as": None},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "RoofTop2", "depends_on": None, "same_drone_as": "SubTask1"},
    {"name": "SubTask3", "skill": "CaptureThermalImage", "object": "House1", "depends_on": None, "same_drone_as": None}
]"""},

# Example 2 --------------------------------------------------------------------------------------------
{"role": "user", "content": "Pick up a payload from House2 and deliver it to House3."},
{"role": "assistant", "content": """# SubTask1: Pick up the payload from House2
# SubTask2: Deliver the package to House3
 
# TEMPORAL DEPENDENCIES (THEN/AFTER/BEFORE)
# SubTask2 depends_on SubTask1  [reason: "delivery after pickup"]

# SAME-DRONE DEPENDENCIES
# SubTask2 same_drone_as SubTask1  [reason: "same carrier keeps payload"]

subtasks = [
    {"name": "SubTask1", "skill": "PickupPayload", "object": "House2", "depends_on": None, "same_drone_as": None},
    {"name": "SubTask2", "skill": "ReleasePayload", "object": "House3", "depends_on": "SubTask1", "same_drone_as": "SubTask1"}
]"""},

# Example 3 --------------------------------------------------------------------------------------------
{"role": "user", "content": "Take an RGB image of each house in the order of House2, House3, House1 with the same drone."},
{"role": "assistant", "content": """# SubTask1: Take RGB image of House2
# SubTask2: Take RGB image of House3
# SubTask3: Take RGB image of House1

# TEMPORAL DEPENDENCIES (THEN/AFTER/BEFORE)
# SubTask2 depends_on SubTask1  [reason: "order"]
# SubTask3 depends_on SubTask2  [reason: "order"]

# SAME-DRONE DEPENDENCIES
# SubTask2 same_drone_as SubTask1  [reason: "with the same drone"]
# SubTask3 same_drone_as SubTask1  [reason: "with the same drone"]

subtasks = [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "House2", "depends_on": None, "same_drone_as": None},
    {"name": "SubTask2", "skill": "CaptureRGBImage", "object": "House3", "depends_on": "SubTask1", "same_drone_as": "SubTask1"},
    {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "House1", "depends_on": "SubTask2", "same_drone_as": "SubTask1"}
]"""},

# Example 4 --------------------------------------------------------------------------------------------
{"role":"user", "content": "Take a thermal image of Tower then take an RGB image of it."},
{"role":"assistant", "content": """# SubTask1: Take thermal image of Tower
# SubTask2: Take RGB image of Tower

 # TEMPORAL DEPENDENCIES (THEN/AFTER/BEFORE)
# SubTask2 depends_on SubTask1  [reason: "then"]

# SAME-DRONE DEPENDENCIES
# (none)

subtasks = [
    {"name": "SubTask1", "skill": "CaptureThermalImage", "object": "Tower", "depends_on": None, "same_drone_as": None},
    {"name": "SubTask2", "skill": "CaptureRGBImage", "object": "Tower", "depends_on": "SubTask1", "same_drone_as": None}
]"""},

# Example 5 --------------------------------------------------------------------------------------------
{"role": "user", "content": "Deliver a payload from RoofTop1 to House2, then take thermal images of SolarPanel1 and SolarPanel2 with the same drone."},
{"role": "assistant", "content": """# SubTask1: Pick up payload from RoofTop1
# SubTask2: Deliver package to House2
# SubTask3: Take thermal image of SolarPanel1
# SubTask4: Take thermal image of SolarPanel2

# TEMPORAL DEPENDENCIES (THEN/AFTER/BEFORE)
# SubTask2 depends_on SubTask1  [reason: "delivery after pickup"]
# SubTask3 depends_on SubTask2  [reason: "then (start imaging after delivery)"]
# SubTask4 depends_on SubTask2  [reason: "then (start imaging after delivery)"]

# SAME-DRONE DEPENDENCIES
# SubTask2 same_drone_as SubTask1  [reason: "same carrier keeps payload"]
# SubTask4 same_drone_as SubTask3  [reason: "with the same drone"]

subtasks = [
    {"name": "SubTask1", "skill": "PickupPayload", "object": "RoofTop1", "depends_on": None, "same_drone_as": None},
    {"name": "SubTask2", "skill": "ReleasePayload", "object": "House2", "depends_on": "SubTask1", "same_drone_as": "SubTask1"},
    {"name": "SubTask3", "skill": "CaptureThermalImage", "object": "SolarPanel1", "depends_on": "SubTask2", "same_drone_as": None},
    {"name": "SubTask4", "skill": "CaptureThermalImage", "object": "SolarPanel2", "depends_on": "SubTask2", "same_drone_as": "SubTask3"}
]
"""}
]
