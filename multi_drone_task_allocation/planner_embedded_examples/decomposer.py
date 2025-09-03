ACTIONS = ["CaptureRGBImage", "CaptureThermalImage", "PickupPayload", "ReleasePayload"]
OBJECTS = ["Base", "RoofTop1", "RoofTop2", "SolarPanel1", "SolarPanel2", "House1", "House2", "House3", "Tower"]

messages = [{"role": "system", "content": f"""You decompose a natural-language drone task into a list of subtasks outputting only Python code.
  
ALLOWED SKILLS: {", ".join(ACTIONS)}
ALLOWED OBJECTS: {", ".join(OBJECTS)}

CONSTRAINTS:
1) Use `depends_on` if the task description specifies an explicit order (e.g., "after", "then").
2) Use `same_drone_as` if the task description specifies that subtasks must be done by the same drone.
3) Subtasks can ONLY set `depends_on` or `same_drone_as` to other subtasks from the same sentence.
4) Any “deliver from X to Y” must include both:
   - a `PickupPayload` subtask at X, and
   - a `ReleasePayload` subtask at Y.  
   The ReleasePayload must depend_on the PickupPayload and use the same drone.

OUTPUT FORMAT:
# <SubTaskNumber>: <short description>
# ...

# DEPENDENCIES
# <brief reasoning lines>

subtasks = [
    {{"name": "<SubTaskNumber>", "skill": "<SkillName>", "object": "<ObjectName>", "depends_on": "<SubTaskNumber>" or None, "same_drone_as": "<SubTaskNumber>" or None}},
    ...
]
"""
},

# Example 1 ------------------------------------------------------------------------------------------
{"role": "user", "content": "Take RGB image of RoofTop1 and thermal image of RoofTop2 with the same drone. Take thermal image of House1."},
{"role": "assistant", "content": """# SubTask1: Take RGB image of RoofTop1
# SubTask2: Take thermal image of RoofTop2
# SubTask3: Take thermal image of House1

# DEPENDENCIES
# There is no dependency between the subtasks.
# SubTask1 and SubTask2 must be executed by the same drone. 

subtasks = [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "RoofTop1", "depends_on": None, "same_drone_as": None},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "RoofTop2", "depends_on": None, "same_drone_as": "SubTask1"},
    {"name": "SubTask3", "skill": "CaptureThermalImage", "object": "House1", "depends_on": None, "same_drone_as": None}
]"""},

# # Example 2 --------------------------------------------------------------------------------------------
# {"role": "user", "content": "Pick up a payload from House2 and deliver it to House3."},
# {"role": "assistant", "content": """# Includes delivery: MUST include PickupPayload and ReleasePayload subtasks
# 
# # SubTask1: Pick up the payload from House2
# # SubTask2: Deliver the package to House3
# 
# # DEPENDENCIES
# 
# # SubTask2 depends on SubTask1 because the delivery can only happen after the pickup.
# # SubTask2 needs to be executed by the same drone as SubTask1 because that is the drone that has the payload already.
# 
# subtasks = [
#     {"name": "SubTask1", "skill": "PickupPayload", "object": "House2", "depends_on": None, "same_drone_as": None},
#     {"name": "SubTask2", "skill": "ReleasePayload", "object": "House3", "depends_on": "SubTask1", "same_drone_as": "SubTask1"}
# ]"""},
# 
# # Example 3 --------------------------------------------------------------------------------------------
# {"role": "user", "content": "Take an RGB image of each house in the order of House2, House3, House1 with the same drone."},
# {"role": "assistant", "content": """# SubTask1: Take RGB image of House2
# # SubTask2: Take RGB image of House3
# # SubTask3: Take RGB image of House1
# 
# # DEPENDENCIES
# 
# # SubTask2 depends on SubTask1 because the picture of House3 needs to be taken after taking one of House2.
# # SubTask3 depends on SubTask2 because the picture of House1 needs to be taken after taking one of House3.
# # SubTask2 and SubTask3 need to be executed by the same drone as SubTask1 because the task description says so.
# 
# subtasks = [
#     {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "House2", "depends_on": None, "same_drone_as": None},
#     {"name": "SubTask2", "skill": "CaptureRGBImage", "object": "House3", "depends_on": "SubTask1", "same_drone_as": "SubTask1"},
#     {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "House1", "depends_on": "SubTask2", "same_drone_as": "SubTask1"}
# ]"""},
# 
# # Example 4 --------------------------------------------------------------------------------------------
# {"role":"user", "content": "Take a thermal image of Tower then take an RGB image of it."},
# {"role":"assistant", "content": """# SubTask1: Take thermal image of Tower
# # SubTask2: Take RGB image of Tower
# 
# # DEPENDENCIES
# 
# # SubTask2 depends on SubTask1 because taking the RGB image is after taking thermal image.
# 
# subtasks = [
#     {"name": "SubTask1", "skill": "CaptureThermalImage", "object": "Tower", "depends_on": None, "same_drone_as": None},
#     {"name": "SubTask2", "skill": "CaptureRGBImage", "object": "Tower", "depends_on": "SubTask1", "same_drone_as": None}
# ]"""},
# 
# # Example 5 --------------------------------------------------------------------------------------------
# {"role": "user", "content": "Deliver a payload from RoofTop1 to House2. Take thermal images of SolarPanel1 and SolarPanel2 with the same drone."},
# {"role": "assistant", "content": """# Includes delivery: MUST include PickupPayload and ReleasePayload subtasks
#  
# # SubTask1: Pick up payload from RoofTop1
# # SubTask2: Deliver package to House2
# # SubTask3: Take thermal image of SolarPanel1
# # SubTask4: Take thermal image of SolarPanel2
# 
# # DEPENDENCIES
# 
# # SubTask2 depends on SubTask1 because the delivery can only happen after the pickup.
# # SubTask2 needs to be executed by the same drone as SubTask1 because that is the drone that has the payload already.
# # SubTask4 needs to be executed by the same drone as SubTask3 because the task description says so.
# # SubTask4 does not depend on SubTask3 as there is no order defined in the task description.
# 
# subtasks = [
#     {"name": "SubTask1", "skill": "PickupPayload", "object": "RoofTop1", "depends_on": None, "same_drone_as": None},
#     {"name": "SubTask2", "skill": "ReleasePayload", "object": "House2", "depends_on": "SubTask1", "same_drone_as": "SubTask1"},
#     {"name": "SubTask3", "skill": "CaptureThermalImage", "object": "SolarPanel1", "depends_on": None, "same_drone_as": None},
#     {"name": "SubTask4", "skill": "CaptureThermalImage", "object": "SolarPanel2", "depends_on": None, "same_drone_as": "SubTask3"}
# ]"""}
]
