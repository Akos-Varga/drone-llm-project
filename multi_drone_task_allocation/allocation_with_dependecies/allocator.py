messages = [{"role": "system", "content": """You are a task allocator for a multi-drone system outputting only Python code.
             
INPUT FORMAT:
schedule = {
  "same_drone_groups": [ {"name": "<GroupName>", "subtasks": ["<SubTaskName>", ...], "skills": ["<SkillName>", ...]}, ... ],
  "rounds": [ {"name": "<RoundName>", "subtasks": ["<SubTaskName>", ...]}, ... ],
  "subtasks": [ {"name": "<SubTaskName>", "skill": "<Skill>", "object": "<ObjectName>"}, ... ]
 }


CONSTRAINTS:
1) Assign each subtask only to a drone that has the required skill.
2) Subtasks within a "same_drone_group" MUST be assigned to the same drone, which must have all skills required by that group.
3) In the same round, one drone MUST do only one subtask, so DO NOT use one drone more than once in one round.
4) If multiple drones qualify, choose the one with the fewest skills. If several drones tie for the fewest skills, select the drone with the smallest ID.
5) For every allocated subtask, provide a single-line explanation in arrow format:
   SubTaskX → [group/round constraints] + [skill requirement] → [chosen drone & reason].
                       
AVAILABLE DRONES AND THEIR SKILLS:
{
  "Drone1": ["CaptureRGBImage"],
  "Drone2": ["CaptureRGBImage"],
  "Drone3": ["CaptureThermalImage"],
  "Drone4": ["CaptureThermalImage"],
  "Drone5": ["CaptureRGBImage", "PickupPayload", "ReleasePayload"],
  "Drone6": ["PickupPayload", "ReleasePayload"],
  "Drone7": ["CaptureRGBImage", "CaptureThermalImage"],
  "Drone8": ["CaptureRGBImage"]
}

OUTPUT FORMAT:
# <SubTaskName> → [group/round constraints] + [skill requirement] → [chosen drone & reason]
# ...
   
allocated_subtasks = [
  {"name": "<SubTaskName>", "drone": "<DroneName>"},
  ...
]
"""},

# Example 1 ------------------------------------------------------------------------------------------
{"role":"user", "content": """
schedule = {
  "same_drone_groups": [
    {"name": "Group1", "subtasks": ["SubTask1", "SubTask2"], "skills": ["CaptureRGBImage", "CaptureThermalImage"]}
  ],
  "rounds": [
    {"name": "Round1", "subtasks": ["SubTask1", "SubTask3"]},
    {"name": "Round2", "subtasks": ["SubTask2"]}
  ],
  "subtasks": [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "RoofTop1"},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "RoofTop2"},
    {"name": "SubTask3", "skill": "CaptureThermalImage", "object": "House1"},
  ]
}"""},
{"role":"assistant", "content": """
# SubTask1 → Group1, union of skills in group: [CaptureRGBImage, CaptureThermalImage] → assigned to Drone7 (only drone with all required skills)
# SubTask2 → Group1 (same as SubTask1) → assigned to Drone7
# SubTask3 → No group, same round as SubTask1 so must use different drone, requires CaptureThermalImage → assigned to Drone3 (fewest skills)

allocated_subtasks = [
 {"name": "SubTask1", "drone": "Drone7"},
 {"name": "SubTask2", "drone": "Drone7"},
 {"name": "SubTask3", "drone": "Drone3"}
]"""},
#
## Example 2 ------------------------------------------------------------------------------------------
#{"role":"user", "content": """
#schedule = {
#  "same_drone_groups": [
#    {"name": "Group1", "subtasks": ["SubTask1", "SubTask2"], "skills": [PickupPayload, ReleasePayload]}
#  ],
#  "rounds": [
#    {"name": "Round1", "subtasks": ["SubTask1"]},
#    {"name": "Round2", "subtasks": ["SubTask2"]}
#  ],
#  "subtasks": [
#    {"name": "SubTask1", "skill": "PickupPayload", "object": "House2"},
#    {"name": "SubTask2", "skill": "ReleasePayload", "object": "House3"}
#  ]
#}
#"""},
#{"role":"assistant", "content": """
## SubTask1 → Group1, union of skills in group: [PickupPayload, ReleasePayload] → assigned to Drone6 (fewest skills with both)
## SubTask2 → Group1 (same as SubTask1) → assigned to Drone6
#
#allocated_subtasks = [
#  {"name": "SubTask1", "drone": "Drone6"},
#  {"name": "SubTask2", "drone": "Drone6"}
#]"""},
#
## Example 3 ------------------------------------------------------------------------------------------
#{"role":"user", "content": """
#schedule = {
#  "same_drone_groups": [
#    {"name": "Group1", "subtasks": ["SubTask1", "SubTask2", "SubTask3"], "skills": [CaptureRGBImage]}
#  ],
#  "rounds": [
#    {"name": "Round1", "subtasks": ["SubTask1"]},
#    {"name": "Round2", "subtasks": ["SubTask2"]},
#    {"name": "Round3", "subtasks": ["SubTask3"]}
#  ],
#  "subtasks": [
#    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "House2"},
#    {"name": "SubTask2", "skill": "CaptureRGBImage", "object": "House3"}, 
#    {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "House1"}
#  ]
#}
#"""},
#{"role":"assistant", "content": """
## SubTask1 → Group1, union of skills in group: [CaptureRGBImage] → assigned to Drone1 (fewest skills)
## SubTask2 → Group1 (same as SubTask1) → assigned to Drone1
## SubTask3 → Group1 (same as SubTask1) → assigned to Drone1
#
#allocated_subtasks = [
#  {"name": "SubTask1", "drone": "Drone1"},
#  {"name": "SubTask2", "drone": "Drone1"},
#  {"name": "SubTask3", "drone": "Drone1"}
#]
#"""},
#
## Example 4 ------------------------------------------------------------------------------------------
#{"role":"user", "content": """
#schedule = {
#  "same_drone_groups": [],
#  "rounds": [
#    {"name": "Round1", "subtasks": ["SubTask1"]},
#    {"name": "Round2", "subtasks": ["SubTask2"]}
#  ],
#  "subtasks": [
#    {"name": "SubTask1", "skill": "CaptureThermalImage", "object": "Tower"},
#    {"name": "SubTask2", "skill": "CaptureRGBImage", "object": "Tower"}
#  ]
#}
#"""},
#{"role":"assistant", "content": """
## SubTask1 → No group, requires CaptureThermalImage → assigned to Drone3 (fewest skills)
## SubTask2 → No group, requires CaptureRGBImage → assigned to Drone1 (fewest skills)
#
# allocated_subtasks = [
#  {"name": "SubTask1", "drone": "Drone3"},
#  {"name": "SubTask2", "drone": "Drone1"}
#]
#"""},
#
## Example 5 ------------------------------------------------------------------------------------------
#{"role":"user", "content": """
#schedule = {
#  "same_drone_groups": [
#    {"name": "Group1", "subtasks": ["SubTask1", "SubTask2"], "skills": [PickupPayload, ReleasePayload]},
#    {"name": "Group2", "subtasks": ["SubTask3", "SubTask4"], "skills": [CaptureThermalImage]}
#  ],
#  "rounds": [
#    {"name": "Round1", "subtasks": ["SubTask1", "SubTask3"]},
#    {"name": "Round2", "subtasks": ["SubTask2", "SubTask4"]}
#  ],
#  "subtasks": [
#    {"name": "SubTask1", "skill": "PickupPayload", "object": "RoofTop1"},
#    {"name": "SubTask2", "skill": "ReleasePayload", "object": "House2"},
#    {"name": "SubTask3", "skill": "CaptureThermalImage", "object": "SolarPanel1"},
#    {"name": "SubTask4", "skill": "CaptureThermalImage", "object": "SolarPanel2"}
#  ]
#}
#"""},
#{"role":"assistant", "content": """
## SubTask1 → Group1, union of skills in group: [PickupPayload, ReleasePayload] → assigned to Drone6 (fewest skills with both)
## SubTask2 → Group1 (same as SubTask1) → assigned to Drone6
## SubTask3 → Group2, same round as SubTask1 so must use different drone, union of skills in group: [CaptureThermalImage] → assigned to Drone3 (fewest skills)
## SubTask4 → Group2 (same as SubTask3) → assigned to Drone3
#
#allocated_subtasks = [
# {"name": "SubTask1", "drone": "Drone6"},
# {"name": "SubTask2", "drone": "Drone6"},
# {"name": "SubTask3", "drone": "Drone3"},
# {"name": "SubTask4", "drone": "Drone3"}
#]
#"""}
]
