ACTIONS = ["CaptureRGBImage", "CaptureThermalImage", "PickupPayload", "ReleasePayload"]
OBJECTS = ["Base", "RoofTop1", "RoofTop2", "SolarPanel1", "SolarPanel2", "House1", "House2", "House3", "Tower"]

header = (
    "You decompose a natural-language drone task into a list of subtasks outputting only Python code.\n\n"
    f"ALLOWED SKILLS: {', '.join(ACTIONS)}\n"
    f"ALLOWED OBJECTS: {', '.join(OBJECTS)}\n\n"
)
messages = [{"role": "system", "content": header + """
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
    {"name": "<SubTaskNumber>", "skill": "<SkillName>", "object": "<ObjectName>", "depends_on": "<SubTaskNumber>" or None, "same_drone_as": "<SubTaskNumber>" or None},
    ...
]
---
DEMONSTRATION EXAMPLES (for learning only, ignore them when solving real inputs):

Example 1
Input:
Take RGB image of RoofTop1 and thermal image of RoofTop2 with the same drone. Take thermal image of House1.

Output:
# SubTask1: Take RGB image of RoofTop1
# SubTask2: Take thermal image of RoofTop2
# SubTask3: Take thermal image of House1

# DEPENDENCIES
# SubTask1 and SubTask2 must be executed by the same drone. 
# SubTask3 is independent.

subtasks = [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "RoofTop1", "depends_on": None, "same_drone_as": None},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "RoofTop2", "depends_on": None, "same_drone_as": "SubTask1"},
    {"name": "SubTask3", "skill": "CaptureThermalImage", "object": "House1", "depends_on": None, "same_drone_as": None}
]

---

Example 2
Input:
Take an RGB image of each house in the order of House2, House3, House1 with the same drone.

Output:
# SubTask1: Take RGB image of House2
# SubTask2: Take RGB image of House3
# SubTask3: Take RGB image of House1

# DEPENDENCIES
# SubTask2 depends on SubTask1, SubTask3 depends on SubTask2. 
# All must use the same drone.

subtasks = [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "House2", "depends_on": None, "same_drone_as": None},
    {"name": "SubTask2", "skill": "CaptureRGBImage", "object": "House3", "depends_on": "SubTask1", "same_drone_as": "SubTask1"},
    {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "House1", "depends_on": "SubTask2", "same_drone_as": "SubTask1"}
]

---

Example 3
Input:
Deliver a payload from RoofTop1 to House2. Take thermal images of SolarPanel1 and SolarPanel2 with the same drone.

Output:
# SubTask1: Pick up payload from RoofTop1
# SubTask2: Deliver package to House2
# SubTask3: Take thermal image of SolarPanel1
# SubTask4: Take thermal image of SolarPanel2

# DEPENDENCIES
# SubTask2 depends on SubTask1 and must use the same drone.
# SubTask4 must use the same drone as SubTask3, but no ordering is required.

subtasks = [
    {"name": "SubTask1", "skill": "PickupPayload", "object": "RoofTop1", "depends_on": None, "same_drone_as": None},
    {"name": "SubTask2", "skill": "ReleasePayload", "object": "House2", "depends_on": "SubTask1", "same_drone_as": "SubTask1"},
    {"name": "SubTask3", "skill": "CaptureThermalImage", "object": "SolarPanel1", "depends_on": None, "same_drone_as": None},
    {"name": "SubTask4", "skill": "CaptureThermalImage", "object": "SolarPanel2", "depends_on": None, "same_drone_as": "SubTask3"}
]

---
END OF DEMONSTRATIONS
"""}
]


#Example 2
#Input:
#Pick up a payload from House2 and deliver it to House3.
#
#Output:
## Includes delivery: MUST include PickupPayload and ReleasePayload subtasks
## SubTask1: Pick up the payload from House2
## SubTask2: Deliver the package to House3
#
## DEPENDENCIES
## SubTask2 depends on SubTask1 and must use the same drone.
#
#subtasks = [
#    {"name": "SubTask1", "skill": "PickupPayload", "object": "House2", "depends_on": None, "same_drone_as": None},
#    {"name": "SubTask2", "skill": "ReleasePayload", "object": "House3", "depends_on": "SubTask1", "same_drone_as": "SubTask1"}
#]
#
#---
#
#---
#
#Example 4
#Input:
#Take a thermal image of Tower then take an RGB image of it.
#
#Output:
## SubTask1: Take thermal image of Tower
## SubTask2: Take RGB image of Tower
#
## DEPENDENCIES
## SubTask2 depends on SubTask1.
#
#subtasks = [
#    {"name": "SubTask1", "skill": "CaptureThermalImage", "object": "Tower", "depends_on": None, "same_drone_as": None},
#    {"name": "SubTask2", "skill": "CaptureRGBImage", "object": "Tower", "depends_on": "SubTask1", "same_drone_as": None}
#]
