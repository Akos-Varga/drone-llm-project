# from task_decompose_conversation_v2 import messages as prompt
# from scheduler import messages as prompt
from allocator import messages as prompt
from test_tasks import tasks, tasks_decomposed, tasks_scheduled

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

# Models -----------------------------------------------------------------

m1 = "qwen2.5-coder:1.5b"
m2 = "qwen2.5-coder:3b"
m3 = "codegemma:7b"
m4 = "qwen2.5-coder:7b"
m5 = "gpt-4o-mini"

# Inference --------------------------------------------------------------

for task_name, task in tasks_scheduled:
    base = prompt
    messages = [*base, {"role": "user", "content": task}]
    print(messages)
    print(task_name + "\nTask description: " + task + "\n")
    start_time = time.time()
    LM(model=m3, messages=messages)
    end_time = time.time()
    print(f"\n--- Inference Time: {end_time - start_time:.2f} seconds ---\n" + "="*90)
