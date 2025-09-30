from multi_drone_task_allocation.drone_vis.drone_visualizer import DroneVisualizer
import matplotlib.pyplot as plt
import os

# your dicts
objects = {
    "Base": (18, 63),
    "RoofTop1": (72, 9),
    "RoofTop2": (41, 56),
    "SolarPanel1": (85, 22),
    "SolarPanel2": (33, 97),
    "House1": (5, 44),
    "House2": (92, 71),
    "House3": (47, 36),
    "Tower": (14, 7)
}

drones = {
  "Drone1": {"skills": ["CaptureThermalImage", "PickupPayload", "ReleasePayload"], "pos": (27, 81), "speed": 15},
  "Drone2": {"skills": ["PickupPayload", "ReleasePayload", "CaptureRGBImage"], "pos": (63, 14), "speed": 18},
  "Drone3": {"skills": ["CaptureRGBImage", "CaptureThermalImage"], "pos": (92, 47), "speed": 12},
  "Drone4": {"skills": ["CaptureRGBImage", "RecordVideo"], "pos": (39, 59), "speed": 19},
  "Drone5": {"skills": ["CaptureThermalImage"], "pos": (8, 23), "speed": 11},
  "Drone6": {"skills": ["PickupPayload", "ReleasePayload"], "pos": (74, 66), "speed": 16}
}

# schedule format:
# "DroneX": [{"object": <ObjectName>, "skill": <SkillName>, "startTime": <sec>, "endTime": <sec>}, ...]
schedule = {
    "Drone1": [
        {"object": "RoofTop2", "skill": "CaptureThermalImage", "startTime": 0, "endTime": 4},
        {"object": "SolarPanel2", "skill": "CaptureThermalImage", "startTime": 5, "endTime": 9},
    ],
    "Drone2": [{"object": "RoofTop1", "skill": "CaptureRGBImage", "startTime": 1, "endTime": 6}],
    "Drone3": [{"object": "House2", "skill": "CaptureRGBImage", "startTime": 2, "endTime": 7}],
    "Drone4": [
        {"object": "House3", "skill": "RecordVideo", "startTime": 0, "endTime": 3},
        {"object": "Base", "skill": "RecordVideo", "startTime": 3.5, "endTime": 7.5},
    ],
    # "Drone5": [{"object": "Tower", "skill": "CaptureThermalImage", "startTime": 0, "endTime": 5}],
    # "Drone6": [
    #     {"object": "Base", "skill": "PickupPayload", "startTime": 1, "endTime": 5},
    #     {"object": "House1", "skill": "ReleasePayload", "startTime": 5.5, "endTime": 9.5},
    # ],
}

schedule2 = {
    "Drone5": [{"object": "Tower", "skill": "CaptureThermalImage", "startTime": 0, "endTime": 5}],
    "Drone6": [
        {"object": "Base", "skill": "PickupPayload", "startTime": 1, "endTime": 5},
        {"object": "House1", "skill": "ReleasePayload", "startTime": 5.5, "endTime": 9.5},
    ]
}

cwd = os.path.dirname(os.path.abspath(__file__))

viz1 = DroneVisualizer(objects, drones)
viz1.set_schedule(schedule)
viz1.animate(dt=0.1, extra_hold=1.5, save_path=os.path.join(cwd, "saved_gifs","part1.gif"))

plt.show()

viz1.set_schedule(schedule2)
viz1.animate(dt=0.1, extra_hold=1.5, save_path=os.path.join(cwd, "saved_gifs","part2.gif"))

plt.show()


