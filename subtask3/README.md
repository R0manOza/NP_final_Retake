# Subtask 3 – Navigation in a pedestrian flow

Robot navigates safely in a pedestrian flow video in **at least two directions**, with visualization.

## Workflow (run in order)

1. **`1_video_to_grayscale.py`**  
   Converts your pedestrian flow video to grayscale.  
   - Put your video in this folder and set `INPUT_VIDEO` at the top of the script (default: `pedestrian_flow.mp4`).  
   - Output: `pedestrian_flow_grayscale.mp4` (and optionally a folder of grayscale frames).

2. **`2_get_targets.py`**  
   Produces the targets the robot will use.  
   - Reads the grayscale video from step 1.  
   - Defines two goals (A and B) for the two directions (e.g. left-middle and right-middle).  
   - Optionally computes optical flow (velocity field) between consecutive frames.  
   - Output: `targets.npz` (goal_A, goal_B, frame size, and flow if enabled).

3. **`3_navigate.py`**  
   Runs the robot in the flow and saves the visualization.  
   - Loads grayscale video and `targets.npz`.  
   - Robot is attracted to the current goal (A or B) and repelled by strong optical flow (pedestrian motion).  
   - Alternates goal A→B and B→A (two directions).  
   - Output: `subtask3_navigation.gif` (and a short matplotlib window with the last frame).

## Dependencies

```bash
pip install -r requirements.txt
```

## Config

- **Step 1**: `INPUT_VIDEO`, `OUTPUT_VIDEO`, `OUTPUT_FRAMES_DIR`.
- **Step 2**: `INPUT_VIDEO` (grayscale), `GOAL_A_FRACTION`, `GOAL_B_FRACTION`, `COMPUTE_OPTICAL_FLOW`.
- **Step 3**: `GRAYSCALE_VIDEO`, `TARGETS_FILE`, robot gains (`K_P`, `K_REP`, `K_D`, `R_SAFE`, `V_MAX`), flow repulsion (`K_REP_FLOW`, `FLOW_REP_THRESHOLD`).

## Written report (exam)

- Model: velocity field from optical flow, saturation (v_max), repulsion from flow / pedestrians.
- Inputs/parameters: optical flow, v_max, R_safe, goal positions.
- Numerical methods: RK-style integration (substeps in 3_navigate.py), optical flow (Farneback).
- Test cases: try different videos and parameters; document where it works and where it fails.
