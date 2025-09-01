from decomposer import messages as decomposer_prompt
from allocator import messages as allocator_prompt
from scheduler import messages as scheduler_prompt

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

# Models -----------------------------------------------------------------

m1 = "qwen2.5-coder:1.5b"
m2 = "qwen2.5-coder:3b"
m3 = "codegemma:7b"
m4 = "qwen2.5-coder:7b"
m5 = "gpt-4o-mini"

# Inference --------------------------------------------------------------
user_task = tasks["Task1"]
model = m5

## Decomposer
decomposer_message = [*decomposer_prompt, {"role": "user", "content": user_task}]
print(decomposer_message)
start_time = time.time()
decomposed_task = LM(model=model, messages=decomposer_message)
end_time = time.time()
print(f"\n--- Inference Time: {end_time - start_time:.2f} seconds ---\n" + "="*90)

## Scheduler
scheduler_message = [*scheduler_prompt, {"role": "user", "content": decomposed_task}]
print(scheduler_message)
start_time = time.time()
scheduled_task = LM(model=model, messages=scheduler_message)
end_time = time.time()
print(f"\n--- Inference Time: {end_time - start_time:.2f} seconds ---\n" + "="*90)

## Allocator
allocator_message = [*allocator_prompt, {"role": "user", "content": scheduled_task}]
print(allocator_message)
start_time = time.time()
allocated_task = LM(model=model, messages=allocator_message)
end_time = time.time()
print(f"\n--- Inference Time: {end_time - start_time:.2f} seconds ---\n" + "="*90)

