import math

def compute_travel_times(subtasks_with_drones, drones):
    def euclidean(p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    
    travel_times = {
        "drone_to_object": {},
        "drone_object_to_object": {}
    }
    
    for info in subtasks_with_drones:
        for drone in info["drones"]:
            if drone not in travel_times["drone_to_object"]:
                travel_times["drone_to_object"][drone] = {}
            dist = euclidean(info["pos"], drones[drone]["pos"])
            time = dist / drones[drone]["speed"]
            travel_times["drone_to_object"][drone][info["object"]] = round(time, 1)
            for info_inner in subtasks_with_drones:
                for drone_inner in info_inner["drones"]:
                    if drone_inner == drone:
                        if drone not in travel_times["drone_object_to_object"]:
                            travel_times["drone_object_to_object"][drone] ={}
                        if info["object"] not in travel_times["drone_object_to_object"][drone]:
                            travel_times["drone_object_to_object"][drone][info["object"]] = {}
                        dist = euclidean(info["pos"], info_inner["pos"])
                        time = dist / drones[drone]["speed"]
                        travel_times["drone_object_to_object"][drone][info["object"]][info_inner["object"]] = round(time, 1)
                        
    return travel_times

if __name__ == "__main__":
    drones = {
    "Drone1": {"skills": ["MeasureWind", "RecordVideo"], "pos": (41, 85), "speed": 12},
    "Drone2": {"skills": ["CaptureRGBImage", "MeasureWind"], "pos": (96, 37), "speed": 18},
    "Drone3": {"skills": ["CaptureThermalImage", "InspectStructure", "CaptureRGBImage"], "pos": (73, 12), "speed": 15}
    }

    subtasks_with_drones = [
        {"name": "SubTask1", "skill": "RecordVideo", "object": "Tower", "pos": (39, 2), "drones": ["Drone1"]},
        {"name": "SubTask2", "skill": "RecordVideo", "object": "House3", "pos": (92, 44), "drones": ["Drone1"]},
        {"name": "SubTask3", "skill": "RecordVideo", "object": "Base", "pos": (48, 99), "drones": ["Drone1"]},
        {"name": "SubTask4", "skill": "CaptureRGBImage", "object": "Tower", "pos": (39, 2), "drones": ["Drone2", "Drone3"]},
        {"name": "SubTask5", "skill": "CaptureRGBImage", "object": "House3", "pos": (92, 44), "drones": ["Drone2", "Drone3"]},
        {"name": "SubTask6", "skill": "CaptureRGBImage", "object": "Base", "pos": (48, 99), "drones": ["Drone2", "Drone3"]}
    ]

    travel_times = compute_travel_times(subtasks_with_drones, drones)
    print(travel_times)
