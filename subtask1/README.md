# Subtask 1 – Path following in a constrained environment

One robot follows a path from A to B without crossing path borders. With a map image, the path **avoids obstacles** (dark regions). This document describes the **mathematical methods**, **where they appear in the code**, **how to run and use** the script, and **how to tweak** every parameter.

---

## 1. Mathematical methods used

### 1.1 Synthetic path (no map)

**Parametric curve** from A to B used when no map is loaded:

- **Linear segment**: **A** + t·(**B** − **A**), with t ∈ [0, 1].
- **Perpendicular deviation**: unit vector **n** = (−Δy, Δx) / ‖**B**−**A**‖; amplitude 0.15·max(‖**B**−**A**‖)·sin(πt).
- **Centerline points**: **c**(t) = **A** + t·(**B** − **A**) + 0.15·max(‖**B**−**A**‖)·sin(πt)·**n**.

So the path is a smooth curve between A and B with one bulge perpendicular to the segment.

---

### 1.2 Image → obstacle grid (with map)

- **Grayscale**: G = (R + G + B) / 3 (only RGB; alpha ignored). Values normalized to [0, 1] if needed.
- **Threshold**: pixel (i, j) is **obstacle** if G(i, j) < τ (default τ = 0.5), else **free**.

Result: binary grid where `True` = obstacle, `False` = free.

---

### 1.3 Obstacle inflation (morphological dilation)

- **Margin**: m = ⌈(path width)/2 + robot radius⌉ + INFLATE_MARGIN (in pixels).
- **Dilation**: obstacles are expanded by a square structuring element of side 2m+1. A cell becomes obstacle if any cell within that square is obstacle.

So the planner sees “fatter” obstacles so the path centerline stays far enough from real obstacles for the path corridor and robot to fit.

---

### 1.4 A* pathfinding

- **Graph**: 2D grid; 8-neighbors (horizontal, vertical, diagonal). Edge cost from (x, y) to (x+dx, y+dy) is √(dx² + dy²) (Euclidean).
- **Cost so far**: g(n) = sum of edge costs from start to node n.
- **Heuristic**: h(n) = √[(n_x − G_x)² + (n_y − G_y)²] (Euclidean distance to goal).
- **Priority**: f(n) = g(n) + h(n). Open set is a min-heap by f; each node expanded at most once.

Output: sequence of grid cells from A to B that avoids (inflated) obstacles and is shortest in the graph.

---

### 1.5 Path simplification

- **Downsampling**: from the A* waypoint list, keep start, end, and every k-th point (e.g. k = max(1, ⌊len/25⌋)).

Fewer points make the subsequent spline smoother and less jagged.

---

### 1.6 Parametric splines (B-spline)

- **Input**: list of 2D points (x_i, y_i).
- **Method**: parametric B-spline **s**(u) = (x(u), y(u)), u ∈ [0, 1], degree k = min(3, n−1), smoothing s = 0 (interpolation).
- **Implementation**: `scipy.interpolate.splprep` (build spline), `splev` (evaluate position and derivative at given u).

The path centerline is this curve; the robot will follow u from 0 to 1.

---

### 1.7 Path corridor (offset curves)

- **Tangent**: **T**(u) = (x′(u), y′(u)) from `splev(..., der=1)`.
- **Unit normal** (pointing left): **N**(u) = (−y′(u), x′(u)) / ‖**T**(u)‖.
- **Left boundary**: **s**(u) + (W/2)·**N**(u).
- **Right boundary**: **s**(u) − (W/2)·**N**(u), with W = path width.

The “path with width” is the strip between these two curves.

---

### 1.8 Robot motion (path following)

- **No ODE**: position is defined geometrically along the spline.
- **Parameter**: u ∈ [0, 1], updated each frame by u ← u + Δu (e.g. Δu = 0.02).
- **Position**: **r**(u) = **s**(u) = `splev(u, tck)`.

So the robot moves along the spline without crossing the path borders by construction.

---

## 2. Where each method is in the code

| Mathematical step | Function / place in code | Approx. lines |
|-------------------|--------------------------|---------------|
| Synthetic path (parametric curve) | `make_synthetic_path_points(A, B, n)` | 30–38 |
| Grayscale + threshold → obstacle grid | `image_to_obstacle_grid(img, threshold)` | 41–49 |
| Obstacle inflation (dilation) | `inflate_obstacles(obstacle, margin)` | 52–57 |
| A* search | `astar(grid, start, goal)` | 60–95 |
| Path simplification (downsample) | `simplify_path(path, step)` | 98–106 |
| B-spline from points | `spline_from_points(x_pts, y_pts)` → `splprep` | 109–117 |
| Path corridor (normals + offsets) | `path_boundaries(x_pts, y_pts, width)`; uses `splev(..., der=1)` and offset formulas | 121–136 |
| Robot position along spline | `splev(u, tck)` in `animate()` | 236–237 |
| Choice: map vs no map, A/B, units | `main()`: `if use_map:` vs `else:` | 141–201 |

**Important variables:**

- **Spline**: `tck` (from `spline_from_points` / `path_boundaries`) is used in `splev(u, tck)` for the centerline and in the animation.
- **Path centerline arrays**: `x_center`, `y_center` (and left/right) come from `path_boundaries`.

---

## 3. How to use the code

### 3.1 Run (first time)

```bash
cd "subtask 1"
pip install -r requirements.txt
python subtask1.py
```

A window opens with:

- **Gray strip** = path corridor (with width).
- **Black curve** = spline centerline from A to B.
- **Green dot** = A (start), **red square** = B (goal).
- **Blue circle** = robot moving from A to B along the spline.

### 3.2 Two modes

| Mode | Set at top of `subtask1.py` | What you get |
|------|----------------------------|--------------|
| **No map** | `MAP_IMAGE = None` | Synthetic curved path in a small world (e.g. x∈[0,4], y∈[0,3]). Same path every run. |
| **With map** | `MAP_IMAGE = "map.png"` (and `map.png` in same folder) | Map as background; obstacles = dark pixels; path from A to B **avoiding** obstacles (A* + spline). |

### 3.3 Using your own map

1. Put the image in the `subtask 1` folder (e.g. `map.png`).
2. Set `MAP_IMAGE = "map.png"` (or your filename).
3. In the `if use_map:` block in `main()`, set **A** and **B** in **pixel coordinates** (x, y):
   - x = 0 is left, x = W is right (e.g. 1152 for 1152×648).
   - y = 0 is bottom, y = H is top (e.g. 648).
4. Choose A and B in **free (light)** areas; otherwise A* may fail and the code falls back to a straight segment.

---

## 4. How to tweak the code

All tunable values are at the top of `subtask1.py` or in the `if use_map:` block in `main()`.

### 4.1 Global parameters (top of file, lines ~18–26)

| Parameter | Default | Effect | Tweak tip |
|-----------|--------|--------|-----------|
| `PATH_WIDTH` | 0.8 | Path corridor width (no-map mode, world units). | Larger = wider strip. |
| `ROBOT_RADIUS` | 0.12 | Robot circle radius (no-map mode). | Must be &lt; half path width. |
| `SPEED` | 0.02 | Increase in spline parameter u per frame. | Larger = faster animation; &lt; 1. |
| `MAP_IMAGE` | `"map.png"` | Map filename or `None`. | `None` = synthetic path only. |
| `OBSTACLE_THRESHOLD` | 0.5 | Grayscale &lt; this → obstacle. | Lower (e.g. 0.35) = only darker pixels are obstacles; higher = more obstacles. |
| `INFLATE_MARGIN` | 3 | Extra pixels added to inflation. | Larger = path stays farther from obstacles. |
| `PATH_WIDTH_PX` | 25 | Path width in pixels (map mode). | Match your map scale; wider = thicker corridor. |
| `ROBOT_RADIUS_PX` | 8 | Robot radius in pixels (map mode). | Smaller = thinner robot along path. |

### 4.2 Map mode only: A and B (in `main()`, ~157–159)

| What | Where | Effect |
|------|--------|--------|
| Start point A | `A = [80.0, 80.0]` | Pixel (x, y) of path start. Must be in free space. |
| End point B | `B = [W - 80.0, H - 80.0]` | Pixel (x, y) of path end. Adjust for your map (e.g. 1152×648 → B = [1072, 568]). |

Change these to any (x, y) inside the image and in free (light) regions.

### 4.3 Synthetic path shape (no-map mode)

- In `make_synthetic_path_points`, the **0.15** factor (line ~36) is the amplitude of the perpendicular bulge. **Larger** = more curved path; **0** = straight line.
- **n=20** (line ~34): number of sample points along the synthetic curve before spline fit. More = slightly smoother; 20 is usually enough.

### 4.4 A* and path smoothness (map mode)

- **Simplification**: in `main()`, `simplify_path(path_pixels, max(1, len(path_pixels) // 25))`. The **25** controls how many waypoints remain; **larger divisor** = fewer points = smoother but possibly cutting corners; **smaller** = more points = follows A* more closely.
- **Spline smoothing**: in `spline_from_points`, `s=0` means interpolating (passing through points). To allow smoothing (not strict interpolation), use a positive `s` in `splprep` (e.g. `s=1` or `s=len(pts)`); then the curve may not pass exactly through waypoints.

### 4.5 Animation

- **SPEED**: already above; 0.02 → about 50 frames to go from u=0 to u=1.
- **interval=40** in `FuncAnimation` (line ~244): milliseconds between frames. **Smaller** = faster animation.
- **frames=200**: total frames per loop. Increase if the path is long and you want more than one full traverse before repeat.

### 4.6 Figure size (map mode)

- In `main()`, `fig_w, fig_h = 11.5, 6.5` when `use_map`. Change to match your screen or aspect (e.g. for 1152×648, ratio ~1.78).

### 4.7 Obstacle detection with your image

If obstacles are **light gray** (high grayscale):

- **Lower** `OBSTACLE_THRESHOLD` (e.g. 0.35) so only darker pixels become obstacles.

If the **floor/path is dark**:

- Either use a **higher** threshold so only very dark pixels are obstacles, or invert the logic in `image_to_obstacle_grid` (e.g. `return gray > threshold` for “light = obstacle”).

---

## 5. Quick reference: run vs tweak

| Goal | Action |
|------|--------|
| Run with default map | `MAP_IMAGE = "map.png"`, put `map.png` in folder, run `python subtask1.py`. |
| Run without map | `MAP_IMAGE = None`, run `python subtask1.py`. |
| Change start/end (map) | Edit `A = [..., ...]` and `B = [..., ...]` in the `if use_map:` block. |
| Path avoids more/larger obstacles | Increase `OBSTACLE_THRESHOLD` or `INFLATE_MARGIN`; or increase `PATH_WIDTH_PX` / `ROBOT_RADIUS_PX`. |
| Smoother path (map) | In `simplify_path(..., len(path_pixels) // 25)`, increase **25** (e.g. 30 or 40). |
| Faster/slower robot | Change `SPEED` (e.g. 0.04 or 0.01) or `interval` in `FuncAnimation`. |
| Different spline behavior | In `spline_from_points`, change `s` in `splprep` (0 = interpolate; &gt;0 = smooth). |

---

## 6. Dependencies

- **numpy**: arrays and math.
- **scipy**: `splprep`, `splev` (splines), `binary_dilation` (inflation).
- **matplotlib**: figure, image display, animation, circle patch.

Install once: `pip install -r requirements.txt`.
