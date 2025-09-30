class ScheduleValidator:
    def __init__(self, drones, travel_times):
        self.drones = drones
        self.travel_time = travel_times
        self.schedule = {}

    def validate_schedule(self, schedule):
        self.schedule = schedule

        # Check if drone has skill
        for drone, tasks in schedule.items():
            for task in tasks:
                if task["skill"] not in self.drones[drone]["skills"]:
                    print(f"{task["skill"]} is not a valid skill for {drone}")
                    return False
        return True


if __name__ == "__main__":
    drones = {
  "Drone1": {"skills": ["CaptureThermalImage"], "pos": (27, 81), "speed": 15},
  "Drone2": {"skills": ["MeasureWind","CaptureRGBImage"], "pos": (63, 14), "speed": 18},
  "Drone3": {"skills": ["CaptureRGBImage", "CaptureThermalImage"], "pos": (92, 47), "speed": 12},
  "Drone4": {"skills": ["CaptureRGBImage", "RecordVideo"], "pos": (39, 59), "speed": 19},
  "Drone5": {"skills": ["CaptureThermalImage", "InspectStructure"], "pos": (8, 23), "speed": 11},
  "Drone6": {"skills": ["MeasureWind"], "pos": (74, 66), "speed": 16}
    }
   # travel_times = 