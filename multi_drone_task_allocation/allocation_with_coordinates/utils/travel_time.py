import math

def compute_travel_times(objects, drones, subtasks_with_drones):
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
            dist = euclidean(objects[info["object"]], drones[drone]["pos"])
            time = dist / drones[drone]["speed"]
            travel_times["drone_to_object"][drone][info["object"]] = round(time, 1)
            for info_inner in subtasks_with_drones:
                for drone_inner in info_inner["drones"]:
                    if drone_inner == drone:
                        if drone not in travel_times["drone_object_to_object"]:
                            travel_times["drone_object_to_object"][drone] ={}
                        if info["object"] not in travel_times["drone_object_to_object"][drone]:
                            travel_times["drone_object_to_object"][drone][info["object"]] = {}
                        dist = euclidean(objects[info["object"]], objects[info_inner["object"]])
                        time = dist / drones[drone]["speed"]
                        travel_times["drone_object_to_object"][drone][info["object"]][info_inner["object"]] = round(time, 1)
                        
    return travel_times

if __name__ == "__main__":
    objects = {
    "Base": (48, 99),
    "RoofTop1": (72, 9),
    "RoofTop2": (41, 56),
    "SolarPanel1": (85, 22),
    "SolarPanel2": (87, 20),
    "House1": (5, 44),
    "House2": (92, 71),
    "House3": (92, 44),
    "Tower": (39, 2)
    }   
    
    drones = {
    "Drone1": {"skills": ["MeasureWind", "RecordVideo"], "pos": (41, 85), "speed": 12},
    "Drone2": {"skills": ["CaptureRGBImage", "MeasureWind"], "pos": (96, 37), "speed": 18},
    "Drone3": {"skills": ["CaptureThermalImage", "InspectStructure", "CaptureRGBImage"], "pos": (73, 12), "speed": 15}
    }

    subtasks_with_drones = [
        {"name": "SubTask1", "skill": "RecordVideo", "object": "Tower", "drones": ["Drone1"]},
        {"name": "SubTask2", "skill": "RecordVideo", "object": "House3", "drones": ["Drone1"]},
        {"name": "SubTask3", "skill": "RecordVideo", "object": "Base", "drones": ["Drone1"]},
        {"name": "SubTask4", "skill": "CaptureRGBImage", "object": "Tower", "drones": ["Drone2", "Drone3"]},
        {"name": "SubTask5", "skill": "CaptureRGBImage", "object": "House3", "drones": ["Drone2", "Drone3"]},
        {"name": "SubTask6", "skill": "CaptureRGBImage", "object": "Base", "drones": ["Drone2", "Drone3"]}
    ]

    travel_times = compute_travel_times(objects, drones, subtasks_with_drones)
    print(travel_times)
