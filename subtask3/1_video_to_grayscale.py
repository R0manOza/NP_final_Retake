"""
Subtask 3 – Step 1: Convert a video to grayscale.
Reads an input video and writes a new video (or folder of frames) in grayscale.
Run this first; then run 2_get_targets.py on the grayscale output.
"""

import cv2
from pathlib import Path
import sys

# --------------- Config ---------------
INPUT_VIDEO = "pedestrians.mp4"   # your pedestrian flow video
OUTPUT_VIDEO = "pedestrian_flow_grayscale.mp4"   # grayscale output (same FPS and size)
# Or save as frames in a folder instead (set to None to only write the video):
OUTPUT_FRAMES_DIR = None   # e.g. "grayscale_frames" to save each frame as an image


def main():
    script_dir = Path(__file__).parent
    input_path = script_dir / INPUT_VIDEO
    if not input_path.exists():
        print("Input video not found:", input_path)
        print("Put your video in the 'subtask 3' folder and set INPUT_VIDEO at the top of this script.")
        sys.exit(1)

    cap = cv2.VideoCapture(str(input_path))
    if not cap.isOpened():
        print("Could not open video:", input_path)
        sys.exit(1)

    fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    out_path = script_dir / OUTPUT_VIDEO
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(str(out_path), fourcc, fps, (w, h), False)  # False = grayscale

    frames_dir = None
    if OUTPUT_FRAMES_DIR:
        frames_dir = script_dir / OUTPUT_FRAMES_DIR
        frames_dir.mkdir(parents=True, exist_ok=True)

    n = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        out.write(gray)
        if frames_dir is not None:
            cv2.imwrite(str(frames_dir / f"frame_{n:05d}.png"), gray)
        n += 1

    cap.release()
    out.release()
    print("Done. Grayscale video saved to:", out_path)
    if frames_dir:
        print("Frames saved to:", frames_dir)
    print("Total frames:", n)


if __name__ == "__main__":
    main()
