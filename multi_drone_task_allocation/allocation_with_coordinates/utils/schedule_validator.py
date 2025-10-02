class ScheduleValidator:
    def __init__(self, objects, drones):
        self.objects = objects
        self.drones = drones

    def validate_schedule(self, schedule, travel_times, subtasks_with_drones):
        # Check if drones and objects are valid
        for drone, tasks in schedule.items():
            if drone not in self.drones:
                print(f"ERROR: {drone} is not in drone list")
                return False, 0
            for task in tasks:
                if task["object"] not in self.objects:
                    print(f"ERROR: Object: {task["object"]} is invalid in {task["name"]}")
                    return False, 0

        # Check if schedule has all subtasks ONCE   
        for subtask_allocator in subtasks_with_drones:
            found = 0
            for _, info in schedule.items():
                for subtask_schedule in info:
                    if subtask_allocator["name"] == subtask_schedule["name"]:
                        found +=1
            if found == 0:
                print(f"ERROR: {subtask_allocator["name"]} is not found in schedule.")
                return False, 0
            if found > 1:
                print(f"ERROR: {subtask_allocator["name"]} is found multiple times in the schedule.")
                return False, 0

        # Check if drone has skill 
        for drone, tasks in schedule.items():
            for task in tasks:
                if task["skill"] not in self.drones[drone]["skills"]:
                    print(f"ERROR: {task["skill"]} is not a valid skill for {drone}")
                    return False, 0

        # Check if timing is good
        maxEnd = 0
        for drone, tasks in schedule.items():
            startObject = ""
            prevEndTime = 0
            for task in tasks:
                endObject = task["object"]
                if prevEndTime > task["startTime"]:
                    print(f"ERROR: Invalid starting time for {task["name"]}")
                    return False, 0
                if startObject == "" and travel_times["drone_to_object"][drone][endObject] != round(task["endTime"] - task["startTime"], 1):
                    print(f"ERROR: Invalid traveltime for {task["name"]}. Expected: {travel_times["drone_to_object"][drone][endObject]} Got: {round(task["endTime"] - task["startTime"], 1)}")
                    return False, 0
                if startObject != "" and travel_times["drone_object_to_object"][drone][startObject][endObject] != round(task["endTime"] - task["startTime"], 1):
                    print(f"ERROR: Invalid traveltime for {task["name"]}. Expected: {travel_times["drone_object_to_object"][drone][startObject][endObject]} Got: {round(task["endTime"] - task["startTime"], 1)}")
                    return False, 0
                startObject = endObject
                prevEndTime = task["endTime"]
            maxEnd = max(maxEnd, prevEndTime)

        return True, maxEnd



if __name__ == "__main__":
    from travel_time import compute_travel_times
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
    "Drone3": {"skills": ["CaptureThermalImage", "InspectStructure", "CaptureRGBImage"], "pos": (73, 12), "speed": 15},
    "Drone4": {"skills": ["InspectStructure"], "pos": (73, 12), "speed": 15}
    }

    subtasks_with_drones = [
        {"name": "SubTask1", "skill": "RecordVideo", "object": "Tower", "drones": ["Drone1"]},
        {"name": "SubTask2", "skill": "RecordVideo", "object": "House3", "drones": ["Drone1"]},
        {"name": "SubTask3", "skill": "RecordVideo", "object": "Base", "drones": ["Drone1"]},
        {"name": "SubTask4", "skill": "CaptureRGBImage", "object": "Tower", "drones": ["Drone2", "Drone3"]},
        {"name": "SubTask5", "skill": "CaptureRGBImage", "object": "House3", "drones": ["Drone2", "Drone3"]},
        {"name": "SubTask6", "skill": "CaptureRGBImage", "object": "Base", "drones": ["Drone2", "Drone3"]}
    ]

    schedule = {
        "Drone1": [
        {"name": "SubTask3", "object": "Base", "skill": "RecordVideo", "startTime": 0.0, "endTime": 1.3},
        {"name": "SubTask2", "object": "House3", "skill": "RecordVideo", "startTime": 1.3, "endTime": 7.2},
        {"name": "SubTask1", "object": "Tower", "skill": "RecordVideo", "startTime": 7.2, "endTime": 12.8},
        ],
        "Drone2": [
        {"name": "SubTask5", "object": "House3", "skill": "CaptureRGBImage", "startTime": 0.0, "endTime": 0.4},
        {"name": "SubTask6", "object": "Base", "skill": "CaptureRGBImage", "startTime": 0.4, "endTime": 4.3},
        ],
        "Drone3": [
        {"name": "SubTask4", "object": "Tower", "skill": "CaptureRGBImage", "startTime": 0.0, "endTime": 2.4},
        ]
    }

    travel_times = compute_travel_times(objects, drones, subtasks_with_drones)

    validator = ScheduleValidator(objects, drones)

    print(validator.validate_schedule(schedule, travel_times, subtasks_with_drones))
