# Subtask 2 – Swarm of robots on a constrained path

Two swarms navigate **one path** in **opposite directions** (A→B and B→A) **at the same time**, without collisions. Uses **map2.png** by default; path has width and is parametrized with splines.

---

## One path or separate paths?

The assignment says: **"A path and its width"** (singular) and **"swarms … navigate from A to B and from B to A without accidents"**. So:

- **One path** = one corridor from A to B (with a given width). Both swarms use this **same** path.
- **Two-way traffic** on that path is possible because the path has **width**: we use a **left lane** (A→B) and a **right lane** (B→A) inside the same corridor, like a two-lane road. So there are not two separate paths—there is one path with two directions of flow.

This implementation uses one spline centerline, one gray corridor, and two lanes (lane offset left for A→B, right for B→A) so the swarms can pass without colliding.

---

## 1. Mathematical methods used

### 1.1 Path extraction (same as Subtask 1)

- **With map**: Grayscale G = (R+G+B)/3; obstacle where G < τ. Inflate obstacles by margin m; A* from A to B on the grid (8-neighbors, cost g(n), heuristic h(n) = Euclidean to goal). Waypoints simplified then fitted with a B-spline.
- **No map**: Synthetic centerline **c**(t) = **A** + t·(**B**−**A**) + 0.15·max(‖**B**−**A**‖)·sin(πt)·**n**, then spline through sample points.

Result: spline **s**(u) = (x(u), y(u)), u ∈ [0, 1], and path width W.

### 1.2 Path corridor and lane offset

- **Unit normal** (left): **N**(u) = (−y′(u), x′(u)) / ‖**T**(u)‖.
- **Lane offset**: each robot follows a curve parallel to the centerline:
  - **A→B** (direction +1): **p**(u) = **s**(u) + (W/2)·**N**(u)·lane_offset with lane_offset &lt; 0 (left side).
  - **B→A** (direction −1): **p**(u) = **s**(u) − (W/2)·**N**(u)·lane_offset with lane_offset &gt; 0 (right side).

So the two swarms use opposite sides of the path (left/right lanes), reducing head-on collisions.

### 1.3 Robot state and motion

- **State** of robot i: (u_i, direction_i, lane_i). u_i ∈ [0, 1] is the spline parameter; direction ∈ {+1, −1}; lane is the offset factor (e.g. ±0.4 for left/right lane).
- **Position**: **r**_i = **s**(u_i) + (W/2)·lane_i·**N**(u_i), from `position_on_path(tck, u, width, lane)`.
- **Desired motion**: u_i ← u_i + direction_i · Δu each frame (same Δu for all; simultaneous start).

No ODE: motion is geometric along the path with a fixed step in u.

### 1.4 Collision avoidance (same direction)

- **Same direction** (e.g. both A→B): cap each robot’s step so it does not get closer than MIN_U_SEP_SAME (in u) to the next robot ahead. So the swarm keeps moving in a train instead of blocking completely.

### 1.5 Collision avoidance (opposite directions)

- **Opposite directions**: require minimum **Euclidean distance** ‖**r**_i − **r**_j‖ ≥ OPPOSITE_SAFE_FACTOR × robot_radius (e.g. ~2.2 so they can pass when on opposite lanes).
- If moving would bring two opposite-direction robots closer than that, the moving robot does not advance that frame.

### 1.6 Simultaneous start

- At t = 0: swarm at A has u = 0, direction = +1; swarm at B has u = 1, direction = −1. Both are updated in the same `animate()` call every frame, so they start and run at the same time.

---

## 2. Where each method is in the code

| Mathematical step | Function / place | Approx. lines |
|-------------------|------------------|---------------|
| Synthetic path | `make_synthetic_path_points` | 52–60 |
| Image → obstacle grid | `image_to_obstacle_grid` | 63–71 |
| Inflate obstacles | `inflate_obstacles` | 74–79 |
| A* pathfinding | `astar` | 82–108 |
| Simplify path | `simplify_path` | 111–119 |
| B-spline from points | `spline_from_points` | 122–130 |
| Path corridor + normals | `path_boundaries` | 133–147 |
| Position on path with lane | `position_on_path(tck, u, width, lane_offset)` | 150–157 |
| Swarm state init (u, dir, lane) | `main()`: building `robots` list | 198–203 |
| Same-direction separation | `animate()`: cap cand_u by MIN_U_SEP_SAME | 233–248 |
| Opposite-direction distance | `animate()`: OPPOSITE_SAFE_FACTOR × robot_radius | 250–257 |
| Update positions and draw | `animate()`: position_on_path, circles[i].center | 243–246 |

---

## 3. How to use the code

### 3.1 Run (first time)

```bash
cd "subtask 2"
pip install -r requirements.txt
python subtask2.py
```

You should see:

- **Gray corridor** = path with width.
- **Black centerline** = spline from A to B.
- **Green dot** = A, **red square** = B.
- **Blue circles** = robots going A→B (left lane).
- **Red circles** = robots going B→A (right lane).
- Both swarms start at the same time and move without overlapping.

### 3.2 Map: map2.png

- Put **map2.png** in the `subtask 2` folder (same idea as subtask 1: dark = obstacles, light = free).
- The script uses **MAP_IMAGE = "map2.png"** by default. If the file is missing, it falls back to a **synthetic path** (no image).
- **A** and **B** (in the `if use_map:` block) are in **pixel coordinates**; adjust for your map size (e.g. 1152×648 → A = [80, 80], B = [W−80, H−80]).

### 3.3 Two modes

| Mode | Set | Result |
|------|-----|--------|
| **With map** | `MAP_IMAGE = "map2.png"` and file exists | Path from A to B avoiding obstacles (A* + spline); path width in pixels. |
| **No map** | `MAP_IMAGE = None` or file missing | Synthetic path in world coordinates (e.g. A=(0,0), B=(4,3)). |

---

## 4. How to tweak the code

All main knobs are at the top of `subtask2.py` or in the `if use_map:` block.

### 4.1 Global parameters (top of file)

| Parameter | Default | Effect | Tweak tip |
|-----------|--------|--------|-----------|
| `MAP_IMAGE` | `"map2.png"` | Map file or `None`. | Use your map filename or `None` for synthetic. |
| `PATH_WIDTH` | 0.8 | Path width (no-map mode). | Same as subtask 1. |
| `ROBOT_RADIUS` | 0.12 | Robot radius (no-map mode). | Same as subtask 1. |
| `SPEED` | 0.015 | Step in u per frame (both directions). | Smaller = slower, safer; larger = faster. |
| `N_ROBOTS_AT_A` | 5 | Number of robots starting at A (A→B). | Increase for a larger swarm. |
| `N_ROBOTS_AT_B` | 5 | Number of robots starting at B (B→A). | Same. |
| `MIN_U_SEP_SAME` | 0.08 | Min u-distance between robots in same direction. | Larger = more spacing; too small = overlap. |
| `OPPOSITE_SAFE_FACTOR` | 2.2 | Min distance (opposite dir) = this × robot_radius. | ~2 allows passing on opposite lanes; larger = more clearance. |
| `LANE_OFFSET` | 0.4 | Fraction of half-width for left/right lane. | 0.4 = clearly visible two lanes; 0.25 was hard to see. |
| `OBSTACLE_THRESHOLD` | 0.5 | Grayscale &lt; this ⇒ obstacle (map mode). | Same as subtask 1. |
| `INFLATE_MARGIN` | 3 | Extra inflation (pixels). | Same as subtask 1. |
| `PATH_WIDTH_PX` | 25 | Path width in pixels (map mode). | Same as subtask 1. |
| `ROBOT_RADIUS_PX` | 8 | Robot radius in pixels (map mode). | Same as subtask 1. |

### 4.2 Map mode: A and B (in `main()`, inside `if use_map:`)

- **A**: start of path in pixel coords (e.g. `[80.0, 80.0]`).
- **B**: end of path (e.g. `[W - 80.0, H - 80.0]`). Adjust to your map size.

### 4.3 Animation

- **SPEED**: already above; same for both swarms.
- **frames=400**: total frames per loop; increase if the path is long and robots need more time to cross.
- **interval=40**: ms between frames; smaller = faster animation.

### 4.4 If robots get stuck

- **Same direction**: increase `MIN_U_SEP_SAME` so they don’t tailgate; or reduce `N_ROBOTS_AT_A` / `N_ROBOTS_AT_B`.
- **Opposite direction**: increase `OPPOSITE_SAFE_FACTOR` or `LANE_OFFSET` so lanes are farther apart; or reduce `SPEED` so they pass more gradually.

---

## 5. Quick reference

| Goal | Action |
|------|--------|
| Run with map | Put **map2.png** in folder, run `python subtask2.py`. |
| Run without map | Set `MAP_IMAGE = None` or remove map2.png. |
| More/fewer robots | Change `N_ROBOTS_AT_A` and `N_ROBOTS_AT_B`. |
| Slower/safer | Decrease `SPEED`; increase `MIN_U_SEP_SAME` and `OPPOSITE_SAFE_FACTOR`. |
| Wider lanes | Increase `LANE_OFFSET` (e.g. 0.35). |
| Different map | Set `MAP_IMAGE = "yourmap.png"` and adjust A, B in the `use_map` block. |

---

## 6. Dependencies

Same as Subtask 1: **numpy**, **scipy**, **matplotlib**. Install with `pip install -r requirements.txt`.
