"""
Subtask 2: Swarm of robots on a constrained path.
- Input: path and its width (from map or synthetic), swarm at A, swarm at B.
- Task: swarms navigate A→B and B→A simultaneously without accidents (collision-free).
- Output: path with width + visualization of both swarms.
"""

import numpy as np
from scipy.interpolate import splprep, splev
from scipy.ndimage import binary_dilation
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import matplotlib.animation as animation
from pathlib import Path
import heapq

# --------------- Parameters ---------------
MAP_IMAGE = "map2.png"   # None = synthetic path; "map2.png" = obstacle-aware path
PATH_WIDTH = 0.8
ROBOT_RADIUS = 0.02
SPEED = 0.015              # spline parameter step per frame (smaller = more cautious)
OBSTACLE_THRESHOLD = 0.5
INFLATE_MARGIN = 3
PATH_WIDTH_PX = 35
ROBOT_RADIUS_PX = 4

# Swarm sizes (number of robots at A and at B)
N_ROBOTS_AT_A = 5         # swarm going A → B
N_ROBOTS_AT_B = 5         # swarm going B → A

# Collision avoidance: min distance (in spline-parameter u) between robots in same direction
MIN_U_SEP_SAME = 0.08
# Opposite direction: min distance = this × robot_radius (use ~2 so they can pass when on opposite lanes)
OPPOSITE_SAFE_FACTOR = 2.2
# Lane offset: fraction of half-width; A→B left, B→A right. 0.4 = clearly visible two lanes on one path
LANE_OFFSET = 0.4


def make_synthetic_path_points(A, B, n=20):
    A, B = np.asarray(A, dtype=float), np.asarray(B, dtype=float)
    t = np.linspace(0, 1, n)
    perp = np.array([-(B - A)[1], (B - A)[0]])
    perp = perp / (np.linalg.norm(perp) + 1e-12)
    curve = (A + t[:, None] * (B - A) +
             0.15 * (B - A).max() * np.sin(np.pi * t)[:, None] * perp)
    return curve[:, 0], curve[:, 1]


def image_to_obstacle_grid(img, threshold=0.5):
    if img.ndim >= 3:
        gray = np.mean(img[..., :3], axis=2)
    else:
        gray = np.asarray(img, dtype=float)
    if gray.max() > 1:
        gray = gray / 255.0
    return gray < threshold


def inflate_obstacles(obstacle, margin):
    if margin <= 0:
        return obstacle
    footprint = np.ones((2 * margin + 1, 2 * margin + 1))
    return binary_dilation(obstacle, structure=footprint)


def astar(grid, start, goal):
    H, W = grid.shape
    start = (int(round(start[0])), int(round(start[1])))
    goal = (int(round(goal[0])), int(round(goal[1])))
    for pt in [start, goal]:
        if not (0 <= pt[0] < W and 0 <= pt[1] < H) or grid[pt[1], pt[0]]:
            return None
    neighbors = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
    g, parent = {start: 0}, {}
    open_set = [(0, start)]
    while open_set:
        _, (x, y) = heapq.heappop(open_set)
        if (x, y) == goal:
            path = []
            cur = goal
            while cur is not None:
                path.append(cur)
                cur = parent.get(cur)
            return list(reversed(path))
        for dx, dy in neighbors:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < W and 0 <= ny < H) or grid[ny, nx]:
                continue
            cost = np.sqrt(dx*dx + dy*dy)
            ng = g[(x, y)] + cost
            if (nx, ny) not in g or ng < g[(nx, ny)]:
                g[(nx, ny)] = ng
                heapq.heappush(open_set, (ng + np.hypot(nx - goal[0], ny - goal[1]), (nx, ny)))
                parent[(nx, ny)] = (x, y)
    return None


def simplify_path(path, step=5):
    if len(path) <= 3:
        return path
    out = [path[0]]
    for i in range(step, len(path) - 1, step):
        out.append(path[i])
    out.append(path[-1])
    return out


def spline_from_points(x_pts, y_pts):
    pts = np.column_stack([np.asarray(x_pts, dtype=float), np.asarray(y_pts, dtype=float)])
    n = len(pts)
    if n < 2:
        return None
    k = min(3, n - 1)
    tck, _ = splprep([pts[:, 0], pts[:, 1]], s=0, k=k)
    return tck


def path_boundaries(x_pts, y_pts, width, n_sample=200):
    tck = spline_from_points(x_pts, y_pts)
    if tck is None:
        return None
    u_fine = np.linspace(0, 1, n_sample)
    x_c, y_c = splev(u_fine, tck)
    dx, dy = splev(u_fine, tck, der=1)
    norm = np.sqrt(dx**2 + dy**2) + 1e-12
    nx, ny = -dy / norm, dx / norm
    half = width / 2
    x_left = x_c + half * nx
    y_left = y_c + half * ny
    x_right = x_c - half * nx
    y_right = y_c - half * ny
    return (x_c, y_c), (x_left, y_left), (x_right, y_right), tck


def position_on_path(tck, u, width, lane_offset):
    """Return (x, y) on path at parameter u; lane_offset in [-1, 1]: negative = left, positive = right."""
    x_c, y_c = splev(u, tck)
    dx, dy = splev(u, tck, der=1)
    norm = np.sqrt(dx*dx + dy*dy) + 1e-12
    nx, ny = -dy / norm, dx / norm
    offset = lane_offset * (width / 2)
    return (x_c + offset * nx, y_c + offset * ny)


def main():
    use_map = False
    map_path = MAP_IMAGE
    if isinstance(map_path, str):
        map_path = Path(__file__).parent / map_path
        use_map = map_path.exists()
        if not use_map:
            map_path = None

    if use_map:
        img = plt.imread(str(map_path))
        H, W = img.shape[0], img.shape[1]
        A = [80.0, 80.0]
        B = [W - 80.0, H - 80.0]
        path_width_px = PATH_WIDTH_PX
        robot_radius_px = ROBOT_RADIUS_PX
        obstacle = image_to_obstacle_grid(img, OBSTACLE_THRESHOLD)
        margin = int(np.ceil(path_width_px / 2 + robot_radius_px)) + INFLATE_MARGIN
        obstacle_inflated = inflate_obstacles(obstacle, margin)
        path_pixels = astar(obstacle_inflated, (A[0], A[1]), (B[0], B[1]))
        if path_pixels is None or len(path_pixels) < 2:
            path_pixels = [(A[0], A[1]), (B[0], B[1])]
        path_pixels = simplify_path(path_pixels, max(1, len(path_pixels) // 25))
        x_pts = [p[0] for p in path_pixels]
        y_pts = [p[1] for p in path_pixels]
        result = path_boundaries(x_pts, y_pts, path_width_px)
        if result is None:
            result = path_boundaries([A[0], B[0]], [A[1], B[1]], path_width_px)
        center, (x_left, y_left), (x_right, y_right), tck = result
        x_center, y_center = center
        path_width_plot = path_width_px
        robot_radius_plot = robot_radius_px
        extent = [0, W, 0, H]
        x_lim, y_lim = (0, W), (0, H)
    else:
        A = [0.0, 0.0]
        B = [4.0, 3.0]
        x_pts, y_pts = make_synthetic_path_points(A, B)
        result = path_boundaries(x_pts, y_pts, PATH_WIDTH)
        center, (x_left, y_left), (x_right, y_right), tck = result
        x_center, y_center = center
        path_width_plot = PATH_WIDTH
        robot_radius_plot = ROBOT_RADIUS
        extent = None
        x_lim = (x_center.min() - 0.5, x_center.max() + 0.5)
        y_lim = (y_center.min() - 0.5, y_center.max() + 0.5)

    # Robot state: list of (u, direction, lane_offset); direction +1 = A→B, -1 = B→A
    # A→B use left lane (-1), B→A use right lane (+1) so they don't collide head-on
    robots = []
    for i in range(N_ROBOTS_AT_A):
        robots.append({"u": 0.0, "dir": 1, "lane": -LANE_OFFSET})   # start at A, go to B, left lane
    for i in range(N_ROBOTS_AT_B):
        robots.append({"u": 1.0, "dir": -1, "lane": LANE_OFFSET})   # start at B, go to A, right lane

    if use_map:
        fig_w, fig_h = 11.5, 6.5
    else:
        fig_w, fig_h = 8, 6
    fig, ax = plt.subplots(1, 1, figsize=(fig_w, fig_h))
    if use_map and map_path:
        ax.imshow(plt.imread(str(map_path)), extent=extent, aspect="equal", origin="lower", alpha=0.85, zorder=0)
    ax.set_aspect("equal")
    ax.set_xlim(x_lim[0], x_lim[1])
    ax.set_ylim(y_lim[0], y_lim[1])
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("Subtask 2: Swarm A↔B on constrained path (collision-free)")

    ax.fill(np.r_[x_left, x_right[::-1]], np.r_[y_left, y_right[::-1]],
            color="lightgray", edgecolor="gray", linewidth=1, alpha=0.7, zorder=1)
    ax.plot(x_center, y_center, "k-", lw=1.5, label="Path centerline", zorder=2)
    ax.plot(A[0], A[1], "go", markersize=12, label="A", zorder=3)
    ax.plot(B[0], B[1], "rs", markersize=12, label="B", zorder=3)

    circles = []
    for i, r in enumerate(robots):
        x, y = position_on_path(tck, r["u"], path_width_plot, r["lane"])
        color = "blue" if r["dir"] == 1 else "red"
        circ = Circle((x, y), robot_radius_plot, color=color, ec="black", lw=1, zorder=4)
        ax.add_patch(circ)
        circles.append(circ)

    def init():
        for i, r in enumerate(robots):
            x, y = position_on_path(tck, r["u"], path_width_plot, r["lane"])
            circles[i].center = (x, y)
        return circles

    def animate(_):
        # Desired u for each robot (full step)
        for i, r in enumerate(robots):
            cand_u = r["u"] + r["dir"] * SPEED
            cand_u = np.clip(cand_u, 0.0, 1.0)
            # Same direction: cap so we don't get closer than MIN_U_SEP_SAME to robot ahead
            if r["dir"] == 1:
                for j in range(len(robots)):
                    if i == j or robots[j]["dir"] != 1:
                        continue
                    if robots[j]["u"] > r["u"]:
                        cand_u = min(cand_u, robots[j]["u"] - MIN_U_SEP_SAME)
            else:  # dir == -1
                for j in range(len(robots)):
                    if i == j or robots[j]["dir"] != -1:
                        continue
                    if robots[j]["u"] < r["u"]:
                        cand_u = max(cand_u, robots[j]["u"] + MIN_U_SEP_SAME)
            # Opposite direction: don't move if that would put us closer than OPPOSITE_SAFE to any opposite robot
            safe = OPPOSITE_SAFE_FACTOR * robot_radius_plot
            for j in range(len(robots)):
                if robots[j]["dir"] == r["dir"]:
                    continue
                xi, yi = position_on_path(tck, cand_u, path_width_plot, r["lane"])
                xj, yj = position_on_path(tck, robots[j]["u"], path_width_plot, robots[j]["lane"])
                if np.hypot(xi - xj, yi - yj) < safe:
                    cand_u = r["u"]  # stay put this frame
                    break
            robots[i]["u"] = cand_u
        for i, r in enumerate(robots):
            x, y = position_on_path(tck, r["u"], path_width_plot, r["lane"])
            circles[i].center = (x, y)
        return circles

    anim = animation.FuncAnimation(fig, animate, init_func=init, frames=400, interval=40, blit=True, repeat=True)
    ax.legend(loc="upper right")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
