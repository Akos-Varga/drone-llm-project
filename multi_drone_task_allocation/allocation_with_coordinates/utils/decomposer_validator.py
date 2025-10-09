def validate_decomposer(decomposed_task, solution, skills):
    # Check number of entries are the same
    if len(solution) != len(decomposed_task):
        print("DECOMPOSER ERROR: Subtask number mismatch")
        return False
    
    # Check if task names are the same
    expected_names = {s["name"] for s in solution}
    actual_names = {s["name"] for s in decomposed_task}
    if expected_names != actual_names:
        print("DECOMPOSER ERROR: Subtask name mismatch")
        return False
    
    for subtask in decomposed_task:
        if subtask["service_time"] != skills[subtask["skill"]]:
            print(f"DECOMPOSER ERROR: Wrong service time for {subtask["name"]}.")
            return False
    
    # Check if skill and objects are the same
    expected_set = {(s["skill"], s["object"]) for s in solution}
    actual_set = {(s["skill"], s["object"]) for s in decomposed_task}
    if expected_set != actual_set:
        print("DECOMPOSER ERROR: Skill-object mismatch")
        return False
    
    print("VALID DECOMPOSITION")
    return True

if __name__ == "__main__":
    solution = [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "SolarPanel1"},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "SolarPanel1"},
    {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "SolarPanel2"},
    {"name": "SubTask4", "skill": "CaptureThermalImage", "object": "SolarPanel2"},
    {"name": "SubTask5", "skill": "CaptureThermalImage", "object": "SolarPanel2"}
    ]
    subtasks = [
    {"name": "SubTask1", "skill": "CaptureRGBImage", "object": "SolarPanel1", "service_time": 2.4},
    {"name": "SubTask2", "skill": "CaptureThermalImage", "object": "SolarPanel1", "service_time": 1},
    {"name": "SubTask3", "skill": "CaptureRGBImage", "object": "SolarPanel2", "service_time": 2.4},
    {"name": "SubTask4", "skill": "CaptureThermalImage", "object": "SolarPanel2", "service_time": 1},
    {"name": "SubTask5", "skill": "CaptureThermalImage", "object": "SolarPanel2", "service_time": 1}
    ]
    skills = {
        "CaptureRGBImage": 2.4,
        "CaptureThermalImage": 1
    }
    print(validate_decomposer(subtasks, solution, skills))
