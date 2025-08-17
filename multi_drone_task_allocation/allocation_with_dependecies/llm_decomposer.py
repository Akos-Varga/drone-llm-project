from task_decompose_conversation_v2 import messages as prompt
# from scheduler import messages as prompt
from test_tasks import tasks

import openai
import time
from ollama import chat
from dotenv import load_dotenv
import os

load_dotenv()
api_key = os.getenv("OPENAI_API_KEY")
client = openai.OpenAI(api_key = api_key)

def LM(model, messages):

    if("gpt" in model):
            response = client.chat.completions.create(
                model=model,
                messages=messages,
                temperature=0.0,
                max_tokens=1500
            )
            output=response.choices[0].message.content
            print(output)
            return output

        
    else:
        response = chat(
          model=model,
          messages=messages,
          stream=True
        )

        output = ""
        for chunk in response:
            content = chunk.message.content
            print(content, end="", flush=True)
            output += content
        return output

# Tasks ------------------------------------------------------------------

#tasks =[("Task1", "Pick up a payload from SolarPanel1 and deliver it to Base."),
#        ("Task2", "Take a thermal image of House2, after that with the same drone take an RGB image of House3."),
#        ("Task3", "Deliver a package from House1 to Tower. Take RGB images of SolarPanel1 and SolarPanel2 with the same drone."),
#        ("Task4", "Take thermal image of both solar panels. Take thermal image of all houses with the same drone."),
#        ("Task5", "Take RGB image of the Tower then with this drone deliver a payload from Tower to House2.")
#        ]



# Models -----------------------------------------------------------------

m1 = "qwen2.5-coder:1.5b"
m2 = "qwen2.5-coder:3b"
m3 = "codegemma:7b"
m4 = "qwen2.5-coder:7b"
m5 = "gpt-4o-mini"

# Inference --------------------------------------------------------------

for task_name, task in tasks:
    base = prompt
    messages = [*base, {"role": "user", "content": task}]
    #print(messages)
    print(task_name + "\nTask description: " + task + "\n")
    start_time = time.time()
    LM(model=m5, messages=messages)
    end_time = time.time()
    print(f"\n--- Inference Time: {end_time - start_time:.2f} seconds ---\n" + "="*90)
