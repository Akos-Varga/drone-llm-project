import math

def compute_travel_times(objects: dict, drones: dict):
    def euclidean(p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    
    travel_times = {
        "drone_to_object": {},
        "drone_object_to_object": {}
    }
    
    # Drone → Object travel times
    for drone, info in drones.items():
        travel_times["drone_to_object"][drone] = {}
        for obj, pos in objects.items():
            dist = euclidean(info["pos"], pos)
            time = dist / info["speed"]
            travel_times["drone_to_object"][drone][obj] = round(time, 1)
    
    # Drone-specific Object → Object travel times
    for drone, info in drones.items():
        travel_times["drone_object_to_object"][drone] = {}
        for obj1, pos1 in objects.items():
            travel_times["drone_object_to_object"][drone][obj1] = {}
            for obj2, pos2 in objects.items():
                if obj1 == obj2:
                    travel_times["drone_object_to_object"][drone][obj1][obj2] = 0.0
                else:
                    dist = euclidean(pos1, pos2)
                    time = dist / info["speed"]
                    travel_times["drone_object_to_object"][drone][obj1][obj2] = round(time, 1)
    
    return travel_times

drones = {
  "Drone1": {"skills": ["CaptureRGBImage", "CaptureThermalImage"], "pos": (23, 77), "speed": 14},
  "Drone2": {"skills": ["CaptureThermalImage"], "pos": (64, 12), "speed": 17},
  "Drone3": {"skills": ["CaptureRGBImage"], "pos": (89, 45), "speed": 11},
  "Drone4": {"skills": ["CaptureRGBImage", "CaptureThermalImage", "InspectStructure"], "pos": (35, 58), "speed": 19},
  "Drone5": {"skills": ["RecordVideo"], "pos": (10, 91), "speed": 13}
}

objects = {
    "House1": (12, 87),
    "RoofTop1": (45, 33),
    "RoofTop2": (78, 62),
    "SolarPanel1": (9, 14),
    "SolarPanel2": (65, 90)
}
'''
objects = {
    "House1": (12, 87),
    "RoofTop1": (45, 33),
    "RoofTop2": (78, 62),
    "SolarPanel1": (9, 14),
    "SolarPanel2": (65, 90)
}

drones = {
  "Drone1": {"skills": ["CaptureThermalImage", "PickupPayload", "ReleasePayload"], "pos": (27, 81), "speed": 15},
  "Drone2": {"skills": ["PickupPayload", "ReleasePayload", "CaptureRGBImage"], "pos": (63, 14), "speed": 18},
  "Drone3": {"skills": ["CaptureRGBImage", "CaptureThermalImage"], "pos": (92, 47), "speed": 12},
  "Drone4": {"skills": ["CaptureRGBImage", "RecordVideo"], "pos": (39, 59), "speed": 19},
  "Drone5": {"skills": ["CaptureThermalImage"], "pos": (8, 23), "speed": 11},
  "Drone6": {"skills": ["PickupPayload", "ReleasePayload"], "pos": (74, 66), "speed": 16}
}
'''

travel_times = compute_travel_times(objects, drones)
print(travel_times)
