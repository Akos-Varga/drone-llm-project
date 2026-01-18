# Large Language Model–Based Multi-Agent Planning for Autonomous Drones

This repository implements an **LLM-based multi-agent task planning and scheduling pipeline** for autonomous drones. The system supports:

- **Offline evaluation** using a **test dataset / simulated world**
- **Online execution** with **real autonomous drones** using ROS 2

The core idea is to use a Large Language Model (LLM) to:
1. **Decompose** a high-level task into subtasks
2. **Allocate** subtasks to multiple drones based on capabilities
3. **Schedule** the subtasks while respecting travel time and constraints

---

## Repository Structure

```
.
├── main_pipeline.py        # Run pipeline on test tasks (no real drones)
├── main.py                 # Run pipeline and execute on real drones (ROS 2)
├── pipeline/               # LLM prompts, validators, schedulers, utilities
├── worlds/
│   ├── test_world/         # Simulated world (skills, objects, drones)
│   └── real_world/         # Real drone configuration and environment
├── test_tasks.py           # Benchmark task dataset
├── publisher.py            # ROS 2 drone interface (PosePublisher)
├── results/                # CSV results and visualizations (generated)
└── README.md
```

---

## Requirements

### General
- **Python ≥ 3.9**
- An environment variable or config allowing access to an LLM (e.g., OpenAI-compatible API)

Install Python dependencies:

```bash
pip install -r requirements.txt
```

(If `requirements.txt` is missing, ensure you have: `numpy`, `matplotlib`, `pandas`, `openai` or compatible client, etc.)

---

## Running the Pipeline on the Test Dataset (No Real Drones)

This mode evaluates the planning system using a **simulated environment** and predefined benchmark tasks.

### Entry Point

```bash
python main_pipeline.py
```

### Common Options

| Argument | Description |
|--------|------------|
| `--model` | LLM model name (default: `gpt-5-mini`) |
| `--task_id` | Run a specific task (e.g., `Task1`) |
| `--save` | Save results to CSV (`results/test_results.csv`) |
| `--vrp` | Compute VRP baseline for comparison (slow) |
| `--visualize` | Generate GIF animation of the schedule |

### Examples

Run **all test tasks**:

```bash
python main_pipeline.py --model gpt-5-mini
```

Run a **single task** with visualization:

```bash
python main_pipeline.py --task_id Task1 --visualize
```

Run with **VRP comparison** and save results:

```bash
python main_pipeline.py --save --vrp
```

### Output

- **Console logs**: decomposed tasks, allocations, schedules
- **CSV results**: `results/test_results.csv`
- **Animations**: `results/animations/TaskX.gif`

---

## Running on Real Autonomous Drones (ROS 2)

This mode executes the planned schedule on **real drones** using ROS 2.

⚠️ **WARNING**: This will command real drones. Ensure a safe flight environment.

### Prerequisites

- **ROS 2 installed** (tested with Humble)
- Drone drivers and middleware running (PX4 / MAVROS / custom bridge)
- Properly configured:
  - `worlds/real_world/skills`
  - `worlds/real_world/objects`
  - `worlds/real_world/drones`
  - Drone namespaces and node mappings (`DRONE_TO_NODE`)

Source ROS 2 before running:

```bash
source /opt/ros/humble/setup.bash
```

---

### Entry Point

```bash
python main.py --task "<TASK DESCRIPTION>"
```

### Required Arguments

| Argument | Description |
|--------|------------|
| `--task` | Natural language task description |

### Optional Arguments

| Argument | Description |
|--------|------------|
| `--model` | LLM model name (default: `gpt-5-mini`) |

---

### Example

```bash
python main.py \
  --model gpt-5-mini \
  --task "Inspect the rooftop and the tower, then return to base"
```

### Execution Flow

1. ROS 2 nodes are created for each drone
2. Initial poses are received
3. LLM pipeline generates a **multi-drone schedule**
4. Each drone:
   - Arms
   - Takes off
   - Executes assigned subtasks
   - Returns to base
   - Lands

Each drone runs in a **separate thread** using a `MultiThreadedExecutor`.

---

## LLM Planning Pipeline

The pipeline consists of three LLM-driven stages:

1. **Decomposer** – converts a natural language task into symbolic subtasks
2. **Allocator** – assigns subtasks to drones based on capabilities
3. **Scheduler** – orders tasks while considering travel times

Validation modules ensure:
- Skill feasibility
- Temporal consistency
- Collision-free schedules

---

## Notes & Tips

- For repeatability, use a fixed model and temperature in `pipeline/utils/inference.py`
- VRP comparison is useful for benchmarking but not required for deployment
- Real-world flight altitude is automatically staggered to avoid collisions

---

## Safety Disclaimer

This software directly controls physical drones. The authors assume **no liability** for damage, injury, or regulatory violations. Always:

- Test in simulation first
- Fly in controlled environments
- Follow local aviation regulations

---

## Citation

If you use this work in academic research, please cite the associated paper or repository.

---

## Contact

For questions or collaboration, please open an issue or contact the repository maintainer.

