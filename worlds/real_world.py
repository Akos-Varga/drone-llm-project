skills = {
    "CaptureRGBImage": 1.8,
    "CaptureThermalImage": 2.2,
    "CaptureZoomImage": 3.4,
    "DualSpectralInspect": 4.7,
    "RecordVideo": 5.0
}

objects = {
    "Base": (0, 0, 0),
    "House1": (2.2, 4.3, 0.5),
    "House2": (2.6, -3.4, 0.7),
    "WindTurbine1": (-4.6, -0.6, 1.4),
    "WindTurbine2": (-3.6, -1.5, 1.5),
    "Tower1": (-1.0, 4.1, 1.6),
    "Tower2": (-1.9, 1.0, 1.6)

}

drones = {
    "Drone1": {
        "pos": (0, 0, 0),
        "skills": ["CaptureRGBImage", "RecordVideo", "CaptureZoomImage"],
        "speed": 1
    },
    "Drone2": {
        "pos": (-1.5, 0, 0),
        "skills": ["CaptureRGBImage", "CaptureThermalImage", "CaptureZoomImage", "RecordVideo", "DualSpectralInspect"],
        "speed": 1
    }
    # "Drone3": {
    #     "pos": (42, 72, 57),
    #     "skills": ["CaptureRGBImage", "CaptureThermalImage", "RecordVideo", "DualSpectralInspect"],
    #     "speed": 15
    # }
}