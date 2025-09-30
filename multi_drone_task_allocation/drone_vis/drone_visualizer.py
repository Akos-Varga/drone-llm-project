from typing import Dict, Tuple, List, Any, Optional

import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.animation import FuncAnimation, PillowWriter


class DroneVisualizer:
    def __init__(self, objects: Dict[str, Tuple[float, float]], drones: Dict[str, Dict[str, Any]], world_size: int = 100):
        self.objects = objects
        self.drones = drones
        self.world_size = world_size

        # Keep track of starting and current positions
        self._start_positions: Dict[str, Tuple[float, float]] = {
            name: tuple(info["pos"]) for name, info in drones.items()
        }
        # Current positions begin at start positions and will persist across schedules
        self._current_positions: Dict[str, Tuple[float, float]] = dict(self._start_positions) # Make a copy of _start_positions at the beginning

        #self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self._segments: Dict[str, List[Dict[str, Any]]] = {}
        self.total_time: float = 0.0
        self._dt: float = 0.1
        self._anim: Optional[FuncAnimation] = None
        self.scheduled_objects = {}
        self.scheduled_drones = {}

    def _assign_colors(self, names: List[str]) -> List[str]:
        palette = [
            "tab:blue", "tab:orange", "tab:green", "tab:red", "tab:purple", "tab:brown",
            "tab:pink", "tab:gray", "tab:olive", "tab:cyan"
        ]
        return [palette[i % len(palette)] for i, _ in enumerate(names)]

    def _build_segments(self, schedule: Dict[str, List[Dict[str, Any]]]):
        """
        Build piecewise-linear segments from current position to object targets
        between start and end times for each drone. Starts from self._current_positions.
        """
        self._segments = {}
        max_t = 0.0

        for drone_name, tasks in schedule.items():
            if drone_name not in self._current_positions:
                # Unknown drone in schedule; skip gracefully
                continue

            # Sort tasks by start time
            tasks_sorted = sorted(tasks, key=lambda t: t["startTime"])
            segs: List[Dict[str, Any]] = []

            # Start from the current position (carried over across schedules)
            current_pos = tuple(self._current_positions[drone_name])

            for t in tasks_sorted:
                obj_name = t["object"]
                start_t = float(t["startTime"])
                end_t = float(t["endTime"])
                target_pos = tuple(self.objects[obj_name])

                # If gap before start, hold position
                prev_end = segs[-1]["end_time"] if segs else 0.0

                if start_t > prev_end:
                    segs.append({
                        "kind": "hold",
                        "start_time": prev_end,
                        "end_time": start_t,
                        "start_pos": current_pos,
                        "end_pos": current_pos,
                    })

                # Move from current_pos to target_pos over [start_t, end_t]
                segs.append({
                    "kind": "move",
                    "start_time": start_t,
                    "end_time": end_t,
                    "start_pos": current_pos,
                    "end_pos": target_pos,
                    "object": obj_name,
                    "skill": t["skill"],
                })
                current_pos = target_pos
                max_t = max(max_t, end_t)

            self._segments[drone_name] = segs

        self.total_time = max_t

        # Pre-advance current positions to end-of-schedule so the next schedule starts there,
        self._advance_current_positions_to_schedule_end()

    def _advance_current_positions_to_schedule_end(self):
        """Advance self._current_positions to the last end_pos of each drone's segments, if any."""
        for drone_name, segs in self._segments.items():
            if segs:
                self._current_positions[drone_name] = tuple(segs[-1]["end_pos"])

    def _pos_on_segment(self, seg: Dict[str, Any], t: float) -> Tuple[float, float]:
        t0, t1 = seg["start_time"], seg["end_time"]
        x0, y0 = seg["start_pos"]
        x1, y1 = seg["end_pos"]

        if t1 <= t0 or t <= t0:
            return (x0, y0)
        if t >= t1:
            return (x1, y1)

        # Linear interpolation
        u = (t - t0) / (t1 - t0)
        return (x0 + u * (x1 - x0), y0 + u * (y1 - y0))

    def _pos_at_time(self, drone_name: str, t: float) -> Tuple[float, float]:
        segs = self._segments.get(drone_name, [])
        if not segs:
            # No segments => just use the persisted current position
            return tuple(self._current_positions[drone_name])
        # Segment containing t
        for seg in segs:
            if seg["start_time"] <= t <= seg["end_time"]:
                return self._pos_on_segment(seg, t)
        # Before first or after last
        if t < segs[0]["start_time"]:
            return segs[0]["start_pos"]
        return segs[-1]["end_pos"]

    def _task_at_time(self, drone_name: str, t: float) -> str:
        """
        Return a short description of what the drone is doing at time t.
        """
        segs = self._segments.get(drone_name, [])
        if not segs:
            return "Idle"
        for seg in segs:
            if seg["start_time"] <= t <= seg["end_time"]:
                if seg["kind"] == "move":
                    obj = seg.get("object", "")
                    skill = seg.get("skill", "")
                    if obj and skill:
                        return f"→ {obj} [{skill}]"
                    if obj:
                        return f"→ {obj}"
                    return "Moving"
                else:
                    return "Holding"
        # Before first or after last segment
        if t < segs[0]["start_time"]:
            return "Holding"
        return "Idle"

    def set_schedule(self, schedule: Dict[str, List[Dict[str, Any]]]):
        # If there were previous segments, ensure we start from their final positions
        self.scheduled_objects = {}
        self.scheduled_drones = {}
        for drone, tasks in schedule.items():
            self.scheduled_drones[drone] = self.drones[drone]
            for task in tasks:
                self.scheduled_objects[task["object"]] = self.objects[task["object"]]

        self._build_segments(schedule)


    def animate(self, dt: float = 0.1, extra_hold: float = 2.0, save_path: Optional[str] = None):
        """
        Animate the schedule. If save_path is provided, saves a GIF at that path.
        dt: time step in seconds (frame interval = 1000*dt ms)
        extra_hold: seconds to hold after last scheduled end time
        """
        self._dt = dt
        total = self.total_time + float(extra_hold)
        frames = int(total / dt) + 1

        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.ax.clear()
        self.ax.set_xlim(0, self.world_size * 1.1)
        self.ax.set_ylim(0, self.world_size * 1.1)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.3)
        self.ax.set_title("Drone & Object Map")
        self.time_text = self.ax.text(0.02, 1.02, "", transform=self.ax.transAxes, va="bottom")

        # Objects as circles + labels
        self.object_patches: Dict[str, Circle] = {}
        object_colors = self._assign_colors(list(self.scheduled_objects.keys()))
        for i, (name, (x, y)) in enumerate(self.scheduled_objects.items()):
            c = Circle((x, y), radius=2.0, alpha=0.6, ec="black", fc=object_colors[i])
            self.ax.add_patch(c)
            self.ax.text(x - 5.0, y - 5.0, name, fontsize=8)
            self.object_patches[name] = c

        # Drones as circles + labels (multi-line: name + current task)
        self.drone_patches: Dict[str, Circle] = {}
        self.drone_labels: Dict[str, Any] = {}
        drone_colors = self._assign_colors(list(self.scheduled_drones.keys()))
        for i, (name, _) in enumerate(self.scheduled_drones.items()):
            x, y = self._current_positions[name]
            c = Circle((x, y), radius=1.5, alpha=0.9, ec="black", fc=drone_colors[i])
            self.ax.add_patch(c)
            lbl = self.ax.text(
                x + 1.8, y + 1.8,
                f"{name}\nIdle",
                fontsize=8, fontweight="bold", linespacing=1.3
            )
            self.drone_patches[name] = c
            self.drone_labels[name] = lbl

        def _update(frame_idx: int):
            t = frame_idx * dt
            artists = []
            for name, patch in self.drone_patches.items():
                # Update position
                x, y = self._pos_at_time(name, t)
                patch.center = (x, y)

                # Update 2-line label: name on first line, task under it
                task = self._task_at_time(name, t)
                lbl = self.drone_labels[name]
                lbl.set_position((x + 1.8, y + 1.8))
                lbl.set_text(f"{name}\n{task}")

                artists.extend([patch, lbl])

            self.time_text.set_text(f"t = {t:0.1f}s")
            artists.append(self.time_text)
            return artists

        self._anim = FuncAnimation(self.fig, _update, frames=frames, interval=int(dt * 1000), blit=True)

        if save_path:
            writer = PillowWriter(fps=int(1.0 / dt))
            self._anim.save(save_path, writer=writer)

        # After creating the animation, lock in end-of-schedule positions so the next schedule starts there.
        self._advance_current_positions_to_schedule_end()

        return self._anim
    

if __name__ == "__main__":
    import os

    # your dicts
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

    schedule = {
        "Drone1": [
            {"object": "RoofTop2", "skill": "CaptureThermalImage", "startTime": 0, "endTime": 4},
            {"object": "SolarPanel2", "skill": "CaptureThermalImage", "startTime": 5, "endTime": 9},
        ],
        "Drone2": [{"object": "RoofTop1", "skill": "CaptureRGBImage", "startTime": 1, "endTime": 6}],
        "Drone3": [{"object": "House2", "skill": "CaptureRGBImage", "startTime": 2, "endTime": 7}],
        "Drone4": [
            {"object": "House3", "skill": "RecordVideo", "startTime": 0, "endTime": 3},
            {"object": "Base", "skill": "RecordVideo", "startTime": 3.5, "endTime": 7.5},
        ],
        # "Drone5": [{"object": "Tower", "skill": "CaptureThermalImage", "startTime": 0, "endTime": 5}],
        # "Drone6": [
        #     {"object": "Base", "skill": "PickupPayload", "startTime": 1, "endTime": 5},
        #     {"object": "House1", "skill": "ReleasePayload", "startTime": 5.5, "endTime": 9.5},
        # ],
    }

    schedule2 = {
        "Drone5": [{"object": "Tower", "skill": "CaptureThermalImage", "startTime": 0, "endTime": 5}],
        "Drone6": [
            {"object": "Base", "skill": "PickupPayload", "startTime": 1, "endTime": 5},
            {"object": "House1", "skill": "ReleasePayload", "startTime": 5.5, "endTime": 9.5},
        ]
    }

    cwd = os.path.dirname(os.path.abspath(__file__))

    viz1 = DroneVisualizer(objects, drones)
    viz1.set_schedule(schedule)
    viz1.animate(dt=0.1, extra_hold=1.5, save_path=os.path.join(cwd, "saved_gifs","part1.gif"))

    plt.show()

    viz1.set_schedule(schedule2)
    viz1.animate(dt=0.1, extra_hold=1.5, save_path=os.path.join(cwd, "saved_gifs","part2.gif"))

    plt.show()
