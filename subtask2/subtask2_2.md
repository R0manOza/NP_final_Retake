# Subtask 2.2 – Independent swarm (IVP + RK4, walls as repulsion)

Robots move by an **IVP** (position + velocity ODE) with **target tracking**, **repulsion from other robots**, and **repulsion from path walls** (same law as robot–robot). Time integration uses **RK4**. Output: **live display** and **.gif** only. This document gives the **mathematical methods**, **where they are in the code**, **how to run and use**, and **how to tweak** every parameter.

---

## 1. Mathematical methods used

### 1.1 Path from map (same idea as Subtask 1)

- **Input**: Map image (e.g. map2.png or map3.png). Grayscale G = (R+G+B)/3; obstacle where G < τ (OBSTACLE_THRESHOLD).
- **Obstacle inflation**: Dilation by margin m so the path stays clear.
- **A***: 8-neighbors, cost g(n), heuristic h(n) = Euclidean to goal; shortest path from A to B on the grid.
- **Simplification**: Downsample waypoints (e.g. every ⌊len/25⌋-th point).
- **Spline**: Parametric B-spline **s**(u) = (x(u), y(u)), u ∈ [0, 1], degree k = min(3, n−1), via `splprep` / `splev`.
- **Path corridor**: Left boundary **L**(u) = **s**(u) + (W/2)**N**(u), right **R**(u) = **s**(u) − (W/2)**N**(u), with **N**(u) = (−y′(u), x′(u)) / ‖**T**(u)‖. Sampled as polylines (x_left, y_left), (x_right, y_right).

The path is **independent** of the robots: it is built once; then robots are deployed and their dynamics use this path only for **wall repulsion** (see below).

### 1.2 Robot state and IVP

- **State** of robot i: position **x**_i = (x_i, y_i) and velocity **v**_i = (vx_i, vy_i).
- **IVP** (per the exam):
  - ẋ_i = **v**_i
  - **v̇**_i = (1/m)(**F**_goal + **F**_rep_robots + **F**_rep_walls + **F**_damp)
- **Velocity saturation** (after each step): **v**_i ← **v**_i · min(1, v_max / ‖**v**_i‖).

So the motion is governed by forces; no pre-assigned lanes.

### 1.3 Target tracking (attraction to goal)

- **F**_goal = k_p · (goal − **x**_i) / ‖goal − **x**_i‖ · min(1, ‖goal − **x**_i‖).
- Goal is **B** for robots starting at A, and **A** for robots starting at B. Same formula for all; only the goal point changes.

### 1.4 Repulsion between robots

- **Same repulsion law** for every pair (and later for walls): if distance d = ‖**x**_i − **x**_j‖ < r_safe, then repulsive force on i from j is
  - **f**_rep = (k_rep / d²) · (**x**_i − **x**_j) / d
  - (magnitude k_rep/d², direction from j toward i).
- **F**_rep_robots = sum over all other robots j of **f**_rep(**x**_i, **x**_j) with parameters k_rep and R_SAFE.

### 1.5 Walls as “virtual robots” (repulsion from path boundaries)

- The **left** and **right** boundaries of the path are polylines (x_left, y_left), (x_right, y_right).
- For the current robot at (x_i, y_i):
  - **Nearest point on left wall**: (x_l, y_l) = argmin on left polyline of distance to (x_i, y_i); call that distance d_l.
  - **Nearest point on right wall**: (x_r, y_r) and d_r similarly.
- **Same repulsion law as robots**: if d_l < WALL_SAFE then add **f**_rep((x_i, y_i), (x_l, y_l)) with k_rep_wall and WALL_SAFE. If d_r < WALL_SAFE then add **f**_rep((x_i, y_i), (x_r, y_r)) with the same parameters.
- So the **walls are treated exactly like other agents** in the RK4: they contribute repulsive forces when the robot is within WALL_SAFE of the nearest point on that wall. There is **no** separate “center” or “path force”; only goal, robot–robot repulsion, wall repulsion, and damping.

### 1.6 Damping

- **F**_damp = −k_d **v**_i. Linear damping so the system can reach a steady behavior without unbounded oscillation.

### 1.7 Nearest point on a polyline

- Given point (x, y) and polyline (x_pts, y_pts) (arrays of length n), compute d²_j = (x − x_pts_j)² + (y − y_pts_j)² for all j; take j* = argmin d²_j; return (x_pts_j*, y_pts_j*, √d²_j*). Used for both left and right wall.

### 1.8 RK4 (Runge–Kutta 4)

- State vector **y** = (x_1, y_1, vx_1, vy_1, x_2, y_2, … ). Right-hand side **dy**/dt = (vx_1, vy_1, ax_1, ay_1, … ) where (ax_i, ay_i) = (1/m)(**F**_i).
- One step: k1 = f(**y**), k2 = f(**y** + (dt/2)k1), k3 = f(**y** + (dt/2)k2), k4 = f(**y** + dt·k3); **y**_new = **y** + (dt/6)(k1 + 2k2 + 2k3 + k4). Then apply velocity saturation to all velocity components.
- **Butcher** idea: one step of RK4 uses four evaluations of the right-hand side; the code uses the classical 4-stage formula.

### 1.9 Deployment (initial conditions)

- Robots at **A**: placed in a square grid (SQUARE_SIZE × SQUARE_SIZE) around A with SPACING; goal = B; initial velocity 0.
- Robots at **B**: same around B; goal = A. All start at the same time (same loop updates everyone).

### 1.10 Stopping and output

- **Stop**: when every robot’s distance to its goal is ≤ GOAL_REACHED_RADIUS, the frame generator stops and the run ends.
- **Output**: live window (interactive mode: draw and flush each frame) and a single **.gif** file (PillowWriter), no .mp4.

---

## 2. Where each method is in the code

| Mathematical step | Function / place | Approx. lines |
|-------------------|------------------|---------------|
| Image → obstacle grid | `image_to_obstacle_grid` | 47–54 |
| Inflate obstacles | `inflate_obstacles` | 57–62 |
| A* pathfinding | `astar` | 65–93 |
| Simplify path | `simplify_path` | 96–104 |
| B-spline from points | `spline_from_points` | 107–115 |
| Path corridor (left/right polylines) | `path_boundaries` | 118–132 |
| Nearest point on polyline | `nearest_on_polyline` | 135–139 |
| Build path from map | `build_path_from_map` | 142–161 |
| Deploy square at A/B | `deploy_square` | 164–171 |
| Repulsive force (robot–robot and robot–wall) | `f_rep` | 174–181 |
| Acceleration (goal + rep robots + rep walls + damp) | `acceleration` | 184–215 |
| Velocity saturation | `velocity_saturate` | 218–224 |
| State ↔ vector | `state_to_vec`, `vec_to_state` | 227–240 |
| Right-hand side for RK4 | `rhs` | 243–253 |
| One RK4 step | `rk4_step` | 256–268 |
| Init, frame gen, animate, stop condition | `init`, `frame_gen`, `animate` | 302–324 |
| Live display + GIF save | `main`: writer loop, draw, flush, grab_frame | 328–341 |

---

## 3. How to use the code

### 3.1 Run (first time)

```bash
cd "subtask 2"
pip install -r requirements.txt
python subtask2_2.py
```

- Place your map (e.g. **map2.png** or **map3.png**) in the `subtask 2` folder and set **MAP_IMAGE** at the top of `subtask2_2.py` to that filename.
- A window opens and the swarm moves **live**; when all robots are close enough to their goals, the run stops.
- If **SAVE_VIDEO = True**, the same run is written to **OUTPUT_VIDEO** (default **subtask22_output.gif**). No .mp4; GIF only.

### 3.2 What you see

- **Gray corridor** = path with width (from spline + boundaries).
- **Black curve** = path centerline.
- **Green dot** = A, **red square** = B.
- **Blue circles** = robots whose goal is B (started at A).
- **Red circles** = robots whose goal is A (started at B).
- Robots are repelled by each other and by the **left/right path walls** (same repulsion law).

### 3.3 Map and path

- **A** and **B** are set in `build_path_from_map` (A = A_PX, B = [W−80, H−80]). Edit there if your map size or start/end differ.
- Path is built once (A*, spline, path_boundaries); walls are the left/right polylines used only for repulsion.

---

## 4. How to tweak the code

All main parameters are at the **top** of `subtask2_2.py`.

### 4.1 Path and map

| Parameter | Default | Effect | Tweak tip |
|-----------|--------|--------|-----------|
| **MAP_IMAGE** | "map3.png" | Map filename in same folder. | Use "map2.png" or your file. |
| **OBSTACLE_THRESHOLD** | 0.5 | Grayscale < this ⇒ obstacle. | Lower ⇒ fewer obstacles; higher ⇒ more. |
| **INFLATE_MARGIN** | 3 | Extra inflation (pixels). | Larger ⇒ path farther from obstacles. |
| **PATH_WIDTH_PX** | 35 | Path corridor width (pixels). | Match your scale. |
| **A_PX** | [80, 80] | Start of path (pixels). | Set to your A. B is W−80, H−80. |

### 4.2 Robot dynamics (IVP)

| Parameter | Default | Effect | Tweak tip |
|-----------|--------|--------|-----------|
| **M** | 1.0 | Mass in **v̇** = **F**/m. | Leave 1 unless you scale forces. |
| **K_P** | 9.0 | Goal attraction strength. | Larger ⇒ faster toward goal. |
| **K_REP** | 300.0 | Robot–robot repulsion strength. | Larger ⇒ stronger separation. |
| **K_D** | 0.3 | Damping −k_d **v**. | Lower ⇒ less drag, higher speed. |
| **R_SAFE** | 25.0 | Robot–robot repulsion when d < this. | Larger ⇒ keep more distance. |
| **V_MAX** | 18.0 | Velocity saturation cap. | Only matters if dynamics reach it. |

### 4.3 Walls (virtual robots)

| Parameter | Default | Effect | Tweak tip |
|-----------|--------|--------|-----------|
| **WALL_SAFE** | 20.0 | Wall repulsion when distance to nearest wall point < this. | Larger ⇒ stay farther from walls. |
| **K_REP_WALL** | 350.0 | Wall repulsion strength (same law as f_rep). | Larger ⇒ stronger push from walls. |

### 4.4 Stopping and output

| Parameter | Default | Effect | Tweak tip |
|-----------|--------|--------|-----------|
| **GOAL_REACHED_RADIUS** | 40.0 | Stop when all robots within this of goal. | Larger ⇒ stop earlier. |
| **SAVE_VIDEO** | True | Whether to write a GIF. | False ⇒ only live window. |
| **OUTPUT_VIDEO** | "subtask22_output.gif" | Output filename. | GIF only; no .mp4. |

### 4.5 Deployment and integration

| Parameter | Default | Effect | Tweak tip |
|-----------|--------|--------|-----------|
| **ROBOT_RADIUS_PX** | 4.0 | Robot circle size in plot. | Visual only. |
| **SQUARE_SIZE** | 2 | 2×2 robots at A and at B. | 3 ⇒ 3×3 per side. |
| **SPACING** | 15.0 | Pixel spacing in deployment square. | Larger ⇒ more spread. |
| **DT** | 0.08 | RK4 time step. | Larger ⇒ faster sim; keep stable. |
| **N_SUBSTEPS** | 5 | RK4 steps per animation frame. | More ⇒ more sim time per frame. |

### 4.6 If robots hug walls or collide

- **Increase WALL_SAFE** or **K_REP_WALL** so walls push harder and earlier.
- **Increase R_SAFE** or **K_REP** so robots keep more distance from each other.

### 4.7 If robots are too slow or too fast

- **Faster**: increase **K_P** or decrease **K_D**; or increase **DT** or **N_SUBSTEPS**.
- **Slower**: decrease **K_P** or increase **K_D**.

---

## 5. Quick reference

| Goal | Action |
|------|--------|
| Run with default map | Put map3.png (or MAP_IMAGE) in folder, run `python subtask2_2.py`. |
| Change map | Set **MAP_IMAGE** and adjust **A_PX** / B in code if needed. |
| Only live, no GIF | Set **SAVE_VIDEO = False**. |
| Different GIF name | Set **OUTPUT_VIDEO** (e.g. "run1.gif"). |
| Keep farther from walls | Increase **WALL_SAFE** and/or **K_REP_WALL**. |
| Stronger robot separation | Increase **R_SAFE** and/or **K_REP**. |
| Stop earlier | Increase **GOAL_REACHED_RADIUS**. |
| More/fewer robots | Change **SQUARE_SIZE** (e.g. 3 for 3×3 per side). |

---

## 6. Dependencies

- **numpy**: arrays and math.
- **scipy**: `splprep`, `splev` (splines), `binary_dilation` (inflation).
- **matplotlib**: figure, image, animation, Circle; **PillowWriter** for GIF (no ffmpeg).

Install: `pip install -r requirements.txt` (same as Subtask 2).
