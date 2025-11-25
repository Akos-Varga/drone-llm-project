skills = {
    "CaptureRGBImage": 1.8,
    "CaptureThermalImage": 2.2,
    "CaptureZoomImage": 3.4,
    "DualSpectralInspect": 4.7,
    "RecordVideo": 5.0
}

objects = {
    "Base": (75, 0, 53),
    "House1": (45, 70, 29),
    "House2": (78, 44, 53),
    "House3": (40, 40, 90),
    "RoofTop1": (4, 99, 39),
    "RoofTop2": (84, 37, 90),
    "SolarPanel1": (84, 82, 21),
    "SolarPanel2": (72, 8, 84),
    "Tower": (53, 91, 57)
}

drones = {
    "Drone1": {
        "pos": (34, 89, 13),
        "skills": ["CaptureRGBImage", "RecordVideo", "CaptureZoomImage"],
        "speed": 19
    },
    "Drone2": {
        "pos": (89, 17, 9),
        "skills": ["CaptureRGBImage", "CaptureThermalImage", "CaptureZoomImage", "RecordVideo", "DualSpectralInspect"],
        "speed": 14
    },
    "Drone3": {
        "pos": (42, 72, 57),
        "skills": ["CaptureRGBImage", "CaptureThermalImage", "RecordVideo", "DualSpectralInspect"],
        "speed": 15
    }
}