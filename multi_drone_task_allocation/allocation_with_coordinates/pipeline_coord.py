from multi_drone_task_allocation.allocation_with_coordinates.decomposer_coord import messages as decomposer_prompt
from multi_drone_task_allocation.allocation_with_coordinates.drone_match_coord import messages as matcher_prompt
from multi_drone_task_allocation.allocation_with_coordinates.allocator_coord import messages as allocator_prompt


from multi_drone_task_allocation.allocation_with_coordinates.test_tasks_coord import tasks

import openai
import time
from ollama import chat
from dotenv import load_dotenv
import os

load_dotenv()
api_key = os.getenv("OPENAI_API_KEY")
client = openai.OpenAI(api_key = api_key)

# LM Inference
def LM(model, messages):
    if("gpt" in model):
      params = {
      "model": model,
      "messages": messages,
      }

      if "gpt-5" not in model.lower():
        params["temperature"] = 0.0
        params["max_tokens"] = 1024

      response = client.chat.completions.create(**params)
      output = response.choices[0].message.content
      print(output)
      return output
    
    else:
        response = chat(
          model=model,
          messages=messages,
          stream=True,
          options={
              "temperature": 0.0
          }
        )

        output = ""
        for chunk in response:
            content = chunk.message.content
            print(content, end="", flush=True)
            output += content
        return output

# Message builder --------------------------------------------------------
def build_message(prompt, content):
     # print(content)
     return [*prompt, {"role": "user", "content": content}]

# Formatting functions ---------------------------------------------------
def remove_comments(commented_text):
     lines = commented_text.splitlines(keepends = True)
     return "".join(line for line in lines if not line.lstrip().startswith("#") and line.strip())

def get_groups(text):
    return text[text.index("groups ="):]

# Models -----------------------------------------------------------------
m1 = "qwen2.5-coder:1.5b"
m2 = "qwen2.5-coder:3b"
m3 = "codegemma:7b"
m4 = "qwen2.5-coder:7b"
m5 = "gpt-4o-mini"
m6 = "gpt-5-mini"

# Actions & Objects and & Drones ------------------------------------------------------------
actions = ["RecordVideo", "CaptureRGBImage", "CaptureThermalImage", "PickupPayload", "ReleasePayload"]

objects = {
    "Base": (18, 63),
    "RoofTop1": (72, 9),
    "RoofTop2": (41, 56),
    "SolarPanel1": (85, 22),
    "SolarPanel2": (33, 97),
    "House1": (5, 44),
    "House2": (92, 71),
    "House3": (47, 36),
    "Tower": (14, 7)
}

drones = {
  "Drone1": {"skills": ["CaptureThermalImage", "PickupPayload", "ReleasePayload"], "pos": (27, 81), "speed": 15},
  "Drone2": {"skills": ["PickupPayload", "ReleasePayload", "CaptureRGBImage"], "pos": (63, 14), "speed": 18},
  "Drone3": {"skills": ["CaptureRGBImage", "CaptureThermalImage"], "pos": (92, 47), "speed": 12},
  "Drone4": {"skills": ["CaptureRGBImage", "RecordVideo"], "pos": (39, 59), "speed": 19},
  "Drone5": {"skills": ["CaptureThermalImage"], "pos": (8, 23), "speed": 11},
  "Drone6": {"skills": ["PickupPayload", "ReleasePayload"], "pos": (74, 66), "speed": 16}
}

# Inference --------------------------------------------------------------
model = m6
for task_id, user_task in list(tasks.items()):
  print(task_id + ": " + user_task)
  ## Decomposer
  decomposer_message = build_message(decomposer_prompt, f"Task: {user_task}\n\nAllowed actions: {actions}\n\nAllowed objects: {objects}")
  # print(decomposer_message)
  start_time = time.time()
  decomposed_task = LM(model=model, messages=decomposer_message)
  cleaned_decomposed_task = remove_comments(decomposed_task)
  end_time = time.time()
  print(f"\n--- Inference Time: {end_time - start_time:.2f} seconds ---\n" + "="*90)

  ## Matcher
  groups = get_groups(cleaned_decomposed_task)
  matcher_message = build_message(matcher_prompt, f"drones = {drones}\n\n{groups}")
  # print(matcher_message)
  start_time = time.time()
  matched_task = LM(model=model, messages=matcher_message)
  cleaned_matched_task = remove_comments(matched_task)
  end_time = time.time()
  print(f"\n--- Inference Time: {end_time - start_time:.2f} seconds ---\n" + "="*90)

  ## Allocator
 # allocator_message = build_message(allocator_prompt, f"fleet = {fleet}\n{cleaned_scheduled_task}")
 # # print(allocator_message)
 # start_time = time.time()
 # allocated_task = LM(model=model, messages=allocator_message)
 # cleaned_allocated_task = remove_comments(allocated_task)
 # end_time = time.time()
 # print(f"\n--- Inference Time: {end_time - start_time:.2f} seconds ---\n" + "="*90)
