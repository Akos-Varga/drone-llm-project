from ollama import chat
import openai
from dotenv import load_dotenv
import os

load_dotenv()
api_key = os.getenv("OPENAI_API_KEY")
client = openai.OpenAI(api_key = api_key)

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
    
if __name__ == "__main__":
   # Models -----------------------------------------------------------------
    m1 = "qwen2.5-coder:1.5b"
    m2 = "qwen2.5-coder:3b"
    m3 = "codegemma:7b"
    m4 = "qwen2.5-coder:7b"
    m5 = "gpt-4o-mini"
    m6 = "gpt-5-mini"

    message = [{"role": "user", "content": "Tell me a joke."}]

    LM(model=m4, messages=message)