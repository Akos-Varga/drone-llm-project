import random

def randomizer(skills=None, objects=None, drones=None):
    skills_mod = {}
    objects_mod = {}
    drones_mod = {}
    if skills:
        for skill, _ in skills.items():
            skills_mod[skill] = round(random.uniform(0.5, 5.0), 1)
    if objects:
        for object, _ in objects.items():
            objects_mod[object] = (random.randint(0, 100), random.randint(0, 100))
    if drones:
        drones_mod = drones
        for drone in drones:
            drones_mod[drone]["pos"] = (random.randint(0, 100), random.randint(0, 100))
            drones_mod[drone]["speed"] = random.randint(10, 20)
    
    return skills_mod, objects_mod, drones_mod
    

if __name__ == "__main__":
    skills = {
        "RecordVideo": 1.8,
        "CaptureRGBImage": 2.6,
        "CaptureThermalImage": 1.2,
        "MeasureWind": 2.9,
        "InspectStructure": 0.7
    }

    objects = {
        "Base": (18, 63),
        "RoofTop1": (72, 9),
        "RoofTop2": (41, 56),
        "SolarPanel1": (85, 22),
        "SolarPanel2": (87, 20),
        "House1": (5, 44),
        "House2": (92, 71),
        "House3": (47, 36),
        "Tower": (14, 7)
    }

    drones = {
    "Drone1": {"skills": ["CaptureThermalImage"], "pos": (27, 81), "speed": 15},
    "Drone2": {"skills": ["MeasureWind","CaptureRGBImage"], "pos": (63, 14), "speed": 18},
    "Drone3": {"skills": ["CaptureRGBImage", "CaptureThermalImage"], "pos": (92, 47), "speed": 12},
    "Drone4": {"skills": ["CaptureRGBImage", "RecordVideo"], "pos": (39, 59), "speed": 19},
    "Drone5": {"skills": ["CaptureThermalImage", "InspectStructure"], "pos": (8, 23), "speed": 11},
    "Drone6": {"skills": ["MeasureWind"], "pos": (74, 66), "speed": 16}
    }

    skills, objects, drones = randomizer(skills, objects, drones)

    print(skills)
    print(objects)
    print(drones)