"""
Subtask 1: Follow a path in a constrained environment.
- Input: map (image or synthetic), points A and B, path width, robot size.
- Task: extract path A->B (avoiding obstacles when map given), parametrize with splines,
        robot moves along path without crossing borders.
- Output: path with width + visualization of robot following the path.
"""

import numpy as np
from scipy.interpolate import splprep, splev
from scipy.ndimage import binary_dilation
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import matplotlib.animation as animation
from pathlib import Path
import heapq

# --------------- Parameters (inputs) ---------------
PATH_WIDTH = 0.8          # width of path corridor (world units if no map; pixels if map)
ROBOT_RADIUS = 0.12       # robot size - radius (world units if no map; pixels if map)
SPEED = 0.02              # animation: advance per frame along spline parameter
MAP_IMAGE = "map2.png"     # set to None for synthetic path; use "map.png" (or your file) for obstacle-aware path

# When using a map: set A and B in PIXEL coordinates (x, y) in the "With map" block below.
OBSTACLE_THRESHOLD = 0.5  # grayscale < this => obstacle (0=black, 1=white)
INFLATE_MARGIN = 3         # extra pixels to inflate obstacles (so path stays clear)
PATH_WIDTH_PX = 25        # path width in pixels (only used when MAP_IMAGE is set)
ROBOT_RADIUS_PX = 8       # robot radius in pixels (only used when MAP_IMAGE is set)


def make_synthetic_path_points(A, B, n=20):
    """Build a smooth curved path from A to B (synthetic centerline points)."""
    A, B = np.asarray(A, dtype=float), np.asarray(B, dtype=float)
    t = np.linspace(0, 1, n)
    perp = np.array([-(B - A)[1], (B - A)[0]])
    perp = perp / (np.linalg.norm(perp) + 1e-12)
    curve = (A + t[:, None] * (B - A) +
             0.15 * (B - A).max() * np.sin(np.pi * t)[:, None] * perp)
    return curve[:, 0], curve[:, 1]


def image_to_obstacle_grid(img, threshold=0.5):
    """Convert image to binary: True = obstacle (dark), False = free."""
    if img.ndim >= 3:
        gray = np.mean(img[..., :3], axis=2)  # use RGB only (ignore alpha if present)
    else:
        gray = np.asarray(img, dtype=float)
    if gray.max() > 1:
        gray = gray / 255.0
    return gray < threshold


def inflate_obstacles(obstacle, margin):
    """Expand obstacles by margin (in grid cells) so path centerline stays away."""
    if margin <= 0:
        return obstacle
    footprint = np.ones((2 * margin + 1, 2 * margin + 1))
    return binary_dilation(obstacle, structure=footprint)


def astar(grid, start, goal):
    """A* on 2D grid. grid: True = obstacle. start, goal: (x,y) in pixel coords. Returns path list of (x,y)."""
    H, W = grid.shape
    start = (int(round(start[0])), int(round(start[1])))
    goal = (int(round(goal[0])), int(round(goal[1])))
    for pt in [start, goal]:
        if not (0 <= pt[0] < W and 0 <= pt[1] < H):
            return None
        if grid[pt[1], pt[0]]:
            return None
    # A* with 8-neighbors
    neighbors = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
    g = {start: 0}
    parent = {}
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
                h = np.hypot(nx - goal[0], ny - goal[1])
                heapq.heappush(open_set, (ng + h, (nx, ny)))
                parent[(nx, ny)] = (x, y)
    return None


def simplify_path(path, step=5):
    """Take every step-th point plus start and end for spline (fewer points = smoother)."""
    if len(path) <= 3:
        return path
    out = [path[0]]
    for i in range(step, len(path) - 1, step):
        out.append(path[i])
    out.append(path[-1])
    return out


def spline_from_points(x_pts, y_pts, smooth=0):
    """Parametrize path with splines. Returns (tck,) for splev."""
    pts = np.column_stack([np.asarray(x_pts, dtype=float), np.asarray(y_pts, dtype=float)])
    n = len(pts)
    if n < 2:
        return None
    k = min(3, n - 1)
    tck, u = splprep([pts[:, 0], pts[:, 1]], s=smooth, k=k)
    return tck


def path_boundaries(x_pts, y_pts, width, n_sample=200):
    """Compute left/right boundary polylines (path corridor) from centerline."""
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


def main():
    # --------------- Input: map, A, B, path width, robot size ---------------
    use_map = False
    map_path = MAP_IMAGE
    if isinstance(map_path, str):
        map_path = Path(__file__).parent / map_path
        use_map = map_path.exists()
        if not use_map:
            map_path = None

    if use_map:
        # --------------- With map: use image coordinates (pixels), extract path avoiding obstacles ---------------
        img = plt.imread(str(map_path))
        if img.ndim == 3:
            H, W = img.shape[0], img.shape[1]
        else:
            H, W = img.shape[0], img.shape[1]
        # A and B in pixel coordinates (x, y). Map is 1152 x 648; set start and end of path.
        A = [80.0, 80.0]
        B = [W - 80.0, H - 80.0]
        path_width_px = PATH_WIDTH_PX
        robot_radius_px = ROBOT_RADIUS_PX

        obstacle = image_to_obstacle_grid(img, OBSTACLE_THRESHOLD)
        margin = int(np.ceil(path_width_px / 2 + robot_radius_px)) + INFLATE_MARGIN
        obstacle_inflated = inflate_obstacles(obstacle, margin)

        # A* in (x,y) = (col, row) -> grid is [row, col] = [y, x]
        path_pixels = astar(obstacle_inflated, (A[0], A[1]), (B[0], B[1]))
        if path_pixels is None or len(path_pixels) < 2:
            # Fallback: straight line (may cross obstacles)
            path_pixels = [ (A[0], A[1]), (B[0], B[1]) ]
        path_pixels = simplify_path(path_pixels, max(1, len(path_pixels) // 25))
        x_pts = [p[0] for p in path_pixels]
        y_pts = [p[1] for p in path_pixels]

        result = path_boundaries(x_pts, y_pts, path_width_px)
        if result is None:
            x_pts, y_pts = [A[0], B[0]], [A[1], B[1]]
            result = path_boundaries(x_pts, y_pts, path_width_px)
        center, (x_left, y_left), (x_right, y_right), tck = result
        x_center, y_center = center
        path_width_plot = path_width_px
        robot_radius_plot = robot_radius_px
        extent = [0, W, 0, H]
        x_lim = (0, W)
        y_lim = (0, H)
    else:
        # --------------- No map: synthetic path in world coordinates ---------------
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

    # --------------- Setup figure ---------------
    if use_map:
        # figure aspect ~ map aspect so image isn't squashed (1152 x 648 typical)
        fig_w, fig_h = 11.5, 6.5
    else:
        fig_w, fig_h = 8, 6
    fig, ax = plt.subplots(1, 1, figsize=(fig_w, fig_h))
    if use_map and map_path is not None:
        img = plt.imread(str(map_path))
        ax.imshow(img, extent=extent, aspect="equal", origin="lower", alpha=0.85, zorder=0)
    ax.set_aspect("equal")
    ax.set_xlim(x_lim[0], x_lim[1])
    ax.set_ylim(y_lim[0], y_lim[1])
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("Subtask 1: Robot following path (spline, path width, no border crossing)")

    # Draw path corridor (filled)
    ax.fill(np.r_[x_left, x_right[::-1]], np.r_[y_left, y_right[::-1]],
            color="lightgray", edgecolor="gray", linewidth=1, alpha=0.7, zorder=1)
    ax.plot(x_center, y_center, "k-", lw=1.5, label="Path centerline (spline)", zorder=2)
    ax.plot(A[0], A[1], "go", markersize=12, label="A", zorder=3)
    ax.plot(B[0], B[1], "rs", markersize=12, label="B", zorder=3)

    robot = Circle((A[0], A[1]), robot_radius_plot, color="blue", ec="darkblue", lw=2, zorder=4)
    ax.add_patch(robot)

    u_current = [0.0]

    def init():
        robot.center = (A[0], A[1])
        return (robot,)

    def animate(_):
        u = u_current[0]
        if u >= 1.0:
            u = 1.0
        x, y = splev(u, tck)
        robot.center = (x, y)
        u_current[0] = min(1.0, u + SPEED)
        return (robot,)

    anim = animation.FuncAnimation(
        fig, animate, init_func=init, frames=200, interval=40, blit=True, repeat=True
    )
    ax.legend(loc="upper right")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
