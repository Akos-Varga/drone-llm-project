def validate_decomposer(decomposed_task, solution):
    # Check number of entries are the same
    if len(solution) != len(decomposed_task):
        return False
    
    # Check if task names are the same
    expected_names = {s["name"] for s in solution}
    actual_names = {s["name"] for s in decomposed_task}
    if expected_names != actual_names:
        return False
    
    # Check if 
    expected_set = {(s["skill"], s["object"]) for s in solution}
    actual_set = {(s["skill"], s["object"]) for s in decomposed_task}
    return expected_set == actual_set

if __name__ == "__main__":
    solution = [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "SolarPanel1"},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "SolarPanel1"},
    {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "SolarPanel2"},
    {"name": "SubTask4", "skill": "CaptureThermalImage", "object": "SolarPanel2"},
    {"name": "SubTask5", "skill": "CaptureThermalImage", "object": "SolarPanel2"}
    ]
    subtasks = [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "SolarPanel1"},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "SolarPanel1"},
    {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "SolarPanel2"},
    {"name": "SubTask4", "skill": "CaptureThermalImage", "object": "SolarPanel2"},
    {"name": "SubTask4", "skill": "CaptureThermalImage", "object": "SolarPanel2"}
    ]
    print(validate_decomposer(subtasks,solution))
