import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import re

csv_path = os.path.join("results", "test_results.csv")
df = pd.read_csv(csv_path)

# For VRP makespan, it is the same per task regardless of model; take a single value per task
vrp_per_task = df.groupby("task_id")["VRP_makespan"].first()

def natural_sort_key(s):
    # Extract number from string
    match = re.search(r'\d+', str(s))
    return int(match.group())

# Sort tasks
llm_pivot = df.pivot_table(
    index="task_id", columns="model", values="LLM_makespan", aggfunc="first"
)
llm_pivot = llm_pivot.reindex(sorted(llm_pivot.index, key=natural_sort_key))

desired_order = ["gpt-5", "gpt-5-mini", "gpt-4o", "gpt-4.1"]

# Reorder columns accordingly
llm_pivot = llm_pivot[desired_order]

# Assemble plotting dataset by concatenating LLM models (in order) plus one VRP column
plot_df = llm_pivot.copy()
plot_df["VRP"] = vrp_per_task.reindex(plot_df.index)

# Inference time

# Pivot inference times
inference_pivot = df.pivot_table(
    index="task_id", columns="model", values="LLM_inference_time", aggfunc="first"
)

# Apply same sorting logic as before
inference_pivot = inference_pivot.reindex(sorted(inference_pivot.index, key=natural_sort_key))
inference_pivot = inference_pivot[desired_order]

# ===== Plotting =====

fig, axes = plt.subplots(
    2, 1, figsize=(max(8, plot_df.shape[0] * 0.9), 12),
    sharex=False
)

# ===== UPPER: LLM vs VRP Makespan =====
ax = axes[0]
n_tasks = plot_df.shape[0]
n_series = plot_df.shape[1]
x = np.arange(n_tasks)
width = 0.8 / n_series

for i, col in enumerate(plot_df.columns):
    ax.bar(x + i * width - (n_series - 1) * width / 2,
           plot_df[col].values, width, label=str(col))

ax.set_xticks(x)
ax.set_xticklabels(plot_df.index.astype(str), rotation=45, ha="right")
ax.grid(True, axis="y", linestyle="--", alpha=0.7)
ax.set_xlabel("Task")
ax.set_ylabel("Makespan (s)")
ax.set_title("LLM and VRP Makespan by Task")
ax.legend(title="Model", bbox_to_anchor=(1.02, 1), loc="upper left")

# ===== LOWER: LLM-only Inference/Makespan =====
ax2 = axes[1]
plot_df_lower = inference_pivot.copy()  # LLM-only (no VRP)

for i, col in enumerate(plot_df_lower.columns):
    ax2.bar(x + i * width - (n_series - 1) * width / 2,
            plot_df_lower[col].values, width, label=str(col))

ax2.set_xticks(x)
ax2.set_xticklabels(plot_df_lower.index.astype(str), rotation=45, ha="right")
ax2.grid(True, axis="y", linestyle="--", alpha=0.7)
ax2.set_xlabel("Task")
ax2.set_ylabel("Inference Time (s)")
ax2.set_title("LLM Inference Time by Task")
ax2.legend(title="Model", bbox_to_anchor=(1.02, 1), loc="upper left")

fig.tight_layout()
fig.savefig(os.path.join("results", "visuals", "llm_combined_plots.jpg"), dpi=200, bbox_inches="tight")
plt.close(fig)
