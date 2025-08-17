import torch

if torch.cuda.is_available():
    gpu_name = torch.cuda.get_device_name(0)
    vram_total = torch.cuda.get_device_properties(0).total_memory / 1024**3  # in GB
    print(f"GPU: {gpu_name}")
    print(f"Total VRAM: {vram_total:.2f} GB")
else:
    print("No CUDA-compatible GPU found.")