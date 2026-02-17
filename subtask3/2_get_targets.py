"""
Subtask 3 – Step 2: From the (grayscale) video, compute targets for the robot.
- Loads the grayscale video (or the one produced by 1_video_to_grayscale.py).
- Optionally computes optical flow (velocity field) for pedestrian motion.
- Outputs two goal positions (A and B) for "at least two directions" of navigation.
- Saves: targets.npz (goal_A, goal_B, flow if computed, frame size, etc.) for use by the navigation script.
"""

import cv2
import numpy as np
from pathlib import Path
import sys

# --------------- Config ---------------
INPUT_VIDEO = "pedestrian_flow_grayscale.mp4"   # grayscale video from step 1 (or original)
OUTPUT_TARGETS = "targets.npz"   # output file for step 3

# Goals for the two directions: (x, y) in pixel coords. Robot will navigate A→B and B→A.
# Set from video size (e.g. opposite corners or sides).
GOAL_A_FRACTION = (0.15, 0.5)   # (x_frac, y_frac) of frame size, e.g. left-middle
GOAL_B_FRACTION = (0.85, 0.5)   # right-middle

# Set to True to compute and save optical flow (velocity field) for each frame (slower).
COMPUTE_OPTICAL_FLOW = True
# Uncompressed save is much faster; file will be larger. Use False for smaller file (slow save).
SAVE_FLOW_UNCOMPRESSED = True


def main():
    script_dir = Path(__file__).parent
    input_path = script_dir / INPUT_VIDEO
    if not input_path.exists():
        print("Input video not found:", input_path)
        print("Run 1_video_to_grayscale.py first, or set INPUT_VIDEO to your video.")
        sys.exit(1)

    cap = cv2.VideoCapture(str(input_path))
    if not cap.isOpened():
        print("Could not open video:", input_path)
        sys.exit(1)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0

    # Goal positions in pixels (for the two directions the robot must navigate)
    goal_A = (int(GOAL_A_FRACTION[0] * w), int(GOAL_A_FRACTION[1] * h))
    goal_B = (int(GOAL_B_FRACTION[0] * w), int(GOAL_B_FRACTION[1] * h))

    flows = [] if COMPUTE_OPTICAL_FLOW else None
    ret, prev = cap.read()
    if not ret:
        print("No frames in video.")
        sys.exit(1)
    if prev.ndim == 3:
        prev_gray = cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY)
    else:
        prev_gray = prev

    frame_count = 1
    print("Processing frames (optical flow)" if COMPUTE_OPTICAL_FLOW else "Reading frames...")
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if frame.ndim == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        if COMPUTE_OPTICAL_FLOW:
            flow = cv2.calcOpticalFlowFarneback(
                prev_gray, gray, None, pyr_scale=0.5, levels=3, winsize=15,
                iterations=3, poly_n=5, poly_sigma=1.2, flags=0
            )
            flows.append(flow)
        prev_gray = gray
        # Log current frame (in-place update if total known)
        if n_frames > 0:
            print(f"\r  Frame {frame_count} / {n_frames}", end="", flush=True)
        else:
            print(f"\r  Frame {frame_count}", end="", flush=True)
        frame_count += 1
    print()  # newline after in-place updates

    cap.release()

    out_path = script_dir / OUTPUT_TARGETS
    if flows is not None:
        print("Done with frames. Converting flow list to array (can be slow for many frames)...")
        flows = np.array(flows, dtype=np.float32)
        if SAVE_FLOW_UNCOMPRESSED:
            print("Saving to disk (uncompressed, faster)...")
            np.savez(
                str(out_path),
                goal_A=np.array(goal_A),
                goal_B=np.array(goal_B),
                width=w,
                height=h,
                n_frames=frame_count,
                flow=flows
            )
        else:
            print("Saving to disk (compressed, may take several minutes)...")
            np.savez_compressed(
                str(out_path),
                goal_A=np.array(goal_A),
                goal_B=np.array(goal_B),
                width=w,
                height=h,
                n_frames=frame_count,
                flow=flows
            )
        print("Saved targets + optical flow to:", out_path)
        print("  goal_A:", goal_A, "  goal_B:", goal_B, "  shape:", flows.shape)
    else:
        np.savez_compressed(
            str(out_path),
            goal_A=np.array(goal_A),
            goal_B=np.array(goal_B),
            width=w,
            height=h,
            n_frames=frame_count
        )
        print("Saved targets to:", out_path)
        print("  goal_A:", goal_A, "  goal_B:", goal_B)


if __name__ == "__main__":
    main()
