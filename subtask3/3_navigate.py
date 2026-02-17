"""
Subtask 3 – Step 3: Robot navigates in the pedestrian flow (at least two directions).
- Loads grayscale video (or frames) and targets from step 1 and 2.
- Robot has goal A and goal B; navigates A→B then B→A (two directions) using:
  - Attraction to current goal
  - Repulsion from "pedestrians" (from optical flow magnitude or simple proxy)
  - Optional: velocity field from optical flow (saturated) to avoid flow
- Output: visualization of robot + pedestrians (video frames).
"""

import numpy as np
import cv2
from pathlib import Path
import sys

# --------------- Config ---------------
GRAYSCALE_VIDEO = "pedestrian_flow_grayscale.mp4"   # from step 1
TARGETS_FILE = "targets.npz"   # from step 2
OUTPUT_VIDEO = "subtask3_navigation.gif"   # or .mp4 if you have a writer

# Robot dynamics (same idea as subtask 2.2)
K_P = 6.0          # goal attraction (higher = faster approach to target)
K_REP = 400.0
K_D = 0.8          # damping (slightly lower so robot keeps more speed)
R_SAFE = 20.0
V_MAX = 25.0       # max speed in px per time unit (needed for crossing frame in time)
ROBOT_RADIUS_PX = 6
DT = 0.05
N_SUBSTEPS = 30    # more substeps per video frame so robot actually moves across the frame

# Pedestrian repulsion: treat high-flow pixels as repulsion sources (like other robots)
USE_FLOW_AS_OBSTACLES = True   # if True, high optical-flow magnitude = repulsion
FLOW_REP_THRESHOLD = 1.2       # flow magnitude above this = "pedestrian" (lower = more sensitive)
K_REP_FLOW = 150.0             # strength of repulsion from each motion pixel
FLOW_WINDOW_RADIUS = 70        # robot "sees" motion in this many pixels around it
FLOW_SAMPLE_STEP = 8           # sample every N pixels in window (smaller = more accurate, slower)
PEDESTRIAN_SAFE = 35.0         # effective "radius" for repulsion from motion blob (like R_SAFE)

# Visual: overlay detected pedestrians (high flow) so you can see what the robot "sees"
SHOW_PEDESTRIAN_OVERLAY = True  # draw red overlay where flow magnitude is high
OVERLAY_ALPHA = 0.45            # opacity of pedestrian overlay (0=off, 1=strong)
OVERLAY_MAG_THRESHOLD = 2.0     # flow magnitude above this shown as red (can match or be lower than FLOW_REP_THRESHOLD)


def load_targets(script_dir):
    path = script_dir / TARGETS_FILE
    if not path.exists():
        print("Targets not found. Run 2_get_targets.py first.")
        sys.exit(1)
    data = np.load(path)
    goal_A = tuple(data["goal_A"])
    goal_B = tuple(data["goal_B"])
    w, h = int(data["width"]), int(data["height"])
    n_frames = int(data["n_frames"])
    flow = data["flow"] if "flow" in data else None
    return goal_A, goal_B, w, h, n_frames, flow


def main():
    script_dir = Path(__file__).parent
    video_path = script_dir / GRAYSCALE_VIDEO
    if not video_path.exists():
        print("Grayscale video not found. Run 1_video_to_grayscale.py first.")
        sys.exit(1)

    goal_A, goal_B, W, H, n_frames, flows = load_targets(script_dir)

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        print("Could not open video:", video_path)
        sys.exit(1)

    # Robot state: one robot for simplicity; alternates goal A→B then B→A
    robot = {"x": float(goal_A[0]), "y": float(goal_A[1]), "vx": 0.0, "vy": 0.0, "goal": goal_B}
    current_goal_is_B = True

    def f_rep(xi, yi, xj, yj, k_rep, r_safe):
        dx, dy = xi - xj, yi - yj
        d = np.hypot(dx, dy) + 1e-12
        if d >= r_safe:
            return 0.0, 0.0
        f = k_rep / (d * d)
        return f * (dx / d), f * (dy / d)

    def flow_repulsion(robot, flow_frame, k_rep_flow, flow_thresh, window_radius, sample_step, r_safe):
        """Repel robot from every high-flow point in a window (each point = pedestrian blob)."""
        if flow_frame is None:
            return 0.0, 0.0
        xi, yi = robot["x"], robot["y"]
        Hf, Wf = flow_frame.shape[:2]
        y0 = max(0, int(yi) - window_radius)
        y1 = min(Hf, int(yi) + window_radius + 1)
        x0 = max(0, int(xi) - window_radius)
        x1 = min(Wf, int(xi) + window_radius + 1)
        Frx, Fry = 0.0, 0.0
        for j in range(y0, y1, sample_step):
            for i in range(x0, x1, sample_step):
                mag = np.hypot(flow_frame[j, i, 0], flow_frame[j, i, 1])
                if mag < flow_thresh:
                    continue
                # This pixel is "pedestrian" – repulsion from (i, j) to robot (same law as robot–robot)
                dx = xi - i
                dy = yi - j
                d = np.hypot(dx, dy) + 1e-12
                if d >= r_safe:
                    continue
                f = k_rep_flow / (d * d)
                # Scale by how strong the motion is (more motion = stronger repulsion)
                f *= min(mag / (flow_thresh + 1e-12), 3.0)
                Frx += f * (dx / d)
                Fry += f * (dy / d)
        return Frx, Fry

    def acceleration(robot, flow_frame, k_p, k_rep, k_d, r_safe, k_rep_flow, flow_thresh):
        xi, yi = robot["x"], robot["y"]
        vx, vy = robot["vx"], robot["vy"]
        gx, gy = robot["goal"][0], robot["goal"][1]
        d_goal = np.hypot(gx - xi, gy - yi) + 1e-12
        Fgx = k_p * (gx - xi) / d_goal * min(1.0, d_goal)
        Fgy = k_p * (gy - yi) / d_goal * min(1.0, d_goal)
        Fdx = -k_d * vx
        Fdy = -k_d * vy
        Frx, Fry = 0.0, 0.0
        if USE_FLOW_AS_OBSTACLES and flow_frame is not None:
            Frx, Fry = flow_repulsion(
                robot, flow_frame, k_rep_flow, flow_thresh,
                FLOW_WINDOW_RADIUS, FLOW_SAMPLE_STEP, PEDESTRIAN_SAFE
            )
        ax = (Fgx + Frx + Fdx) / 1.0
        ay = (Fgy + Fry + Fdy) / 1.0
        return ax, ay

    def saturate(vx, vy, v_max):
        v = np.hypot(vx, vy) + 1e-12
        if v <= v_max:
            return vx, vy
        return vx * (v_max / v), vy * (v_max / v)

    frames_out = []
    frame_idx = 0
    flow_idx = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if frame.ndim == 2:
            vis = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            vis = frame.copy()

        flow_frame = flows[flow_idx] if flows is not None and flow_idx < len(flows) else None

        # Visual: overlay where we detect "pedestrians" (high flow) so you can see what the robot sees
        if SHOW_PEDESTRIAN_OVERLAY and flow_frame is not None:
            mag = np.hypot(flow_frame[:, :, 0], flow_frame[:, :, 1]).astype(np.float32)
            mag = cv2.GaussianBlur(mag, (5, 5), 1.0)  # light smooth so blobs are visible
            mask = (mag > OVERLAY_MAG_THRESHOLD).astype(np.uint8)
            overlay = np.zeros_like(vis)
            overlay[mask > 0] = [0, 0, 255]  # BGR red = detected motion (pedestrian)
            vis = cv2.addWeighted(overlay, OVERLAY_ALPHA, vis, 1.0 - OVERLAY_ALPHA, 0)

        for _ in range(N_SUBSTEPS):
            ax, ay = acceleration(robot, flow_frame, K_P, K_REP, K_D, R_SAFE, K_REP_FLOW, FLOW_REP_THRESHOLD)
            robot["vx"] += ax * DT
            robot["vy"] += ay * DT
            robot["vx"], robot["vy"] = saturate(robot["vx"], robot["vy"], V_MAX)
            robot["x"] += robot["vx"] * DT
            robot["y"] += robot["vy"] * DT
            robot["x"] = np.clip(robot["x"], ROBOT_RADIUS_PX, W - 1 - ROBOT_RADIUS_PX)
            robot["y"] = np.clip(robot["y"], ROBOT_RADIUS_PX, H - 1 - ROBOT_RADIUS_PX)

        d_to_goal = np.hypot(robot["x"] - robot["goal"][0], robot["y"] - robot["goal"][1])
        if d_to_goal < 30:
            robot["goal"] = goal_A if current_goal_is_B else goal_B
            current_goal_is_B = not current_goal_is_B

        cx, cy = int(robot["x"]), int(robot["y"])
        cv2.circle(vis, (cx, cy), ROBOT_RADIUS_PX, (0, 255, 0), 2)
        cv2.circle(vis, goal_A, 8, (0, 255, 0), -1)
        cv2.circle(vis, goal_B, 8, (0, 0, 255), -1)
        if SHOW_PEDESTRIAN_OVERLAY:
            cv2.putText(vis, "Red = detected motion (pedestrians)", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        frames_out.append(cv2.cvtColor(vis, cv2.COLOR_BGR2RGB))
        frame_idx += 1
        if flows is not None:
            flow_idx = min(flow_idx + 1, len(flows) - 1)

    cap.release()

    # Write output (GIF via imageio, or fallback: save frames as PNGs)
    try:
        import imageio
        out_path = script_dir / OUTPUT_VIDEO
        imageio.mimsave(str(out_path), frames_out, fps=20, loop=0)
        print("Saved to", out_path)
    except Exception as e:
        print("GIF save failed:", e)
        # Fallback: save each frame as PNG in a folder
        frames_dir = script_dir / "subtask3_frames"
        frames_dir.mkdir(parents=True, exist_ok=True)
        for i, fr in enumerate(frames_out):
            cv2.imwrite(str(frames_dir / f"frame_{i:05d}.png"), cv2.cvtColor(fr, cv2.COLOR_RGB2BGR))
        print("Frames saved as PNGs in:", frames_dir)
        print("For GIF output, install imageio:  pip install imageio")

    # Show last frame
    if frames_out:
        import matplotlib.pyplot as plt
        plt.imshow(frames_out[-1])
        plt.axis("off")
        plt.title("Subtask 3: Robot (green) and goals (green/red)")
        plt.show()


if __name__ == "__main__":
    main()
