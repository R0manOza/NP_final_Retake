"""
Subtask 2.2: Independent swarm with IVP + RK4.
- Path from map (A*, spline, path with width). Robots deployed in squares at A and B.
- Motion: ẋ = v, v̇ = (1/m)(F_goal + F_rep_robots + F_rep_walls + F_damp); velocity saturation.
- Walls of the path act like other robots: same repulsion law when robot is near left/right boundary.
- Output: live display + save as .gif only.
"""

import numpy as np
from scipy.interpolate import splprep, splev
from scipy.ndimage import binary_dilation
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import matplotlib.animation as animation
from pathlib import Path
import heapq

# --------------- Path ---------------
MAP_IMAGE = "map3.png"
OBSTACLE_THRESHOLD = 0.5
INFLATE_MARGIN = 3
PATH_WIDTH_PX = 35
A_PX = [80.0, 80.0]

# --------------- Robot dynamics ---------------
M = 1.0
K_P = 9.0
K_REP = 300.0
K_D = 0.0
R_SAFE = 25.0
V_MAX = 18.0

# Walls = same repulsion as robots (path boundaries act as virtual robots)
WALL_SAFE = 10.0        # repulsion from wall when distance < this (like R_SAFE for walls)
K_REP_WALL = 650.0      # can tune separately from robot repulsion

GOAL_REACHED_RADIUS = 40.0
SAVE_VIDEO = True
OUTPUT_VIDEO = "subtask22_output.gif"   

ROBOT_RADIUS_PX = 4.0
SQUARE_SIZE = 2
SPACING = 15.0
DT = 0.08
N_SUBSTEPS = 5


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


def nearest_on_polyline(x, y, x_pts, y_pts):
    """Nearest point on polyline (x_pts, y_pts) to (x, y). Returns (x_n, y_n, dist)."""
    d2 = (x_pts - x)**2 + (y_pts - y)**2
    i = np.argmin(d2)
    return x_pts[i], y_pts[i], np.sqrt(d2[i])


def build_path_from_map(map_path):
    img = plt.imread(str(map_path))
    H, W = img.shape[0], img.shape[1]
    B = [W - 80.0, H - 80.0]
    A = list(A_PX)
    obstacle = image_to_obstacle_grid(img, OBSTACLE_THRESHOLD)
    margin = int(np.ceil(PATH_WIDTH_PX / 2) + 10) + INFLATE_MARGIN
    obstacle_inflated = inflate_obstacles(obstacle, margin)
    path_pixels = astar(obstacle_inflated, (A[0], A[1]), (B[0], B[1]))
    if path_pixels is None or len(path_pixels) < 2:
        path_pixels = [(A[0], A[1]), (B[0], B[1])]
    path_pixels = simplify_path(path_pixels, max(1, len(path_pixels) // 25))
    x_pts = [p[0] for p in path_pixels]
    y_pts = [p[1] for p in path_pixels]
    result = path_boundaries(x_pts, y_pts, PATH_WIDTH_PX)
    if result is None:
        result = path_boundaries([A[0], B[0]], [A[1], B[1]], PATH_WIDTH_PX)
    (x_c, y_c), (x_left, y_left), (x_right, y_right), tck = result
    return A, B, tck, (x_c, y_c), (x_left, y_left), (x_right, y_right), (W, H), img


def deploy_square(center, goal, n_side, spacing):
    robots = []
    for i in range(n_side):
        for j in range(n_side):
            x = center[0] + (i - (n_side - 1) / 2) * spacing
            y = center[1] + (j - (n_side - 1) / 2) * spacing
            robots.append({"x": x, "y": y, "vx": 0.0, "vy": 0.0, "goal": goal})
    return robots


def f_rep(xi, yi, xj, yj, k_rep, r_safe):
    """Repulsive force on (xi,yi) from (xj,yj). Same law for robot-robot and robot-wall."""
    dx, dy = xi - xj, yi - yj
    d = np.hypot(dx, dy) + 1e-12
    if d >= r_safe:
        return 0.0, 0.0
    f = k_rep / (d * d)
    return f * (dx / d), f * (dy / d)


def acceleration(robot, robots, goal, x_left, y_left, x_right, y_right, k_p, k_rep, k_rep_wall, k_d, r_safe, wall_safe, m):
    xi, yi = robot["x"], robot["y"]
    vx, vy = robot["vx"], robot["vy"]
    gx, gy = goal[0], goal[1]
    # Target
    d_goal = np.hypot(gx - xi, gy - yi) + 1e-12
    Fgx = k_p * (gx - xi) / d_goal * min(1.0, d_goal)
    Fgy = k_p * (gy - yi) / d_goal * min(1.0, d_goal)
    # Repulsion from other robots
    Frx, Fry = 0.0, 0.0
    for other in robots:
        if other is robot:
            continue
        fx, fy = f_rep(xi, yi, other["x"], other["y"], k_rep, r_safe)
        Frx += fx
        Fry += fy
    # Repulsion from walls (same law as robots: walls = virtual robots)
    xl, yl, dl = nearest_on_polyline(xi, yi, x_left, y_left)
    if dl < wall_safe:
        fx, fy = f_rep(xi, yi, xl, yl, k_rep_wall, wall_safe)
        Frx += fx
        Fry += fy
    xr, yr, dr = nearest_on_polyline(xi, yi, x_right, y_right)
    if dr < wall_safe:
        fx, fy = f_rep(xi, yi, xr, yr, k_rep_wall, wall_safe)
        Frx += fx
        Fry += fy
    # Damping
    Fdx = -k_d * vx
    Fdy = -k_d * vy
    ax = (Fgx + Frx + Fdx) / m
    ay = (Fgy + Fry + Fdy) / m
    return ax, ay


def velocity_saturate(vx, vy, v_max):
    v = np.hypot(vx, vy) + 1e-12
    if v <= v_max:
        return vx, vy
    s = v_max / v
    return vx * s, vy * s


def state_to_vec(robots):
    out = []
    for r in robots:
        out.extend([r["x"], r["y"], r["vx"], r["vy"]])
    return np.array(out, dtype=float)


def vec_to_state(robots, vec):
    for i, r in enumerate(robots):
        j = i * 4
        r["x"], r["y"] = vec[j], vec[j+1]
        r["vx"], r["vy"] = vec[j+2], vec[j+3]


def rhs(vec, robots, goals, x_left, y_left, x_right, y_right):
    vec_to_state(robots, vec)
    out = np.zeros_like(vec)
    for i, r in enumerate(robots):
        ax, ay = acceleration(r, robots, goals[i], x_left, y_left, x_right, y_right,
                              K_P, K_REP, K_REP_WALL, K_D, R_SAFE, WALL_SAFE, M)
        j = i * 4
        out[j], out[j+1] = r["vx"], r["vy"]
        out[j+2], out[j+3] = ax, ay
    return out


def rk4_step(vec, dt, robots, goals, x_left, y_left, x_right, y_right):
    k1 = rhs(vec, robots, goals, x_left, y_left, x_right, y_right)
    vec_to_state(robots, vec + 0.5*dt*k1)
    k2 = rhs(vec, robots, goals, x_left, y_left, x_right, y_right)
    vec_to_state(robots, vec + 0.5*dt*k2)
    k3 = rhs(vec, robots, goals, x_left, y_left, x_right, y_right)
    vec_to_state(robots, vec + dt*k3)
    k4 = rhs(vec, robots, goals, x_left, y_left, x_right, y_right)
    vec_to_state(robots, vec)
    new_vec = vec + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
    for i in range(len(robots)):
        j = i * 4
        vx, vy = velocity_saturate(new_vec[j+2], new_vec[j+3], V_MAX)
        new_vec[j+2], new_vec[j+3] = vx, vy
    return new_vec


def main():
    map_path = Path(__file__).parent / MAP_IMAGE
    if not map_path.exists():
        print("Map not found:", MAP_IMAGE)
        return
    A, B, tck, (x_c, y_c), (x_left, y_left), (x_right, y_right), (W, H), img = build_path_from_map(map_path)
    robot_radius = ROBOT_RADIUS_PX

    robots_at_A = deploy_square(A, B, SQUARE_SIZE, SPACING)
    robots_at_B = deploy_square(B, A, SQUARE_SIZE, SPACING)
    robots = robots_at_A + robots_at_B
    goals = [r["goal"] for r in robots]

    fig, ax = plt.subplots(1, 1, figsize=(11.5, 6.5))
    ax.imshow(img, extent=[0, W, 0, H], aspect="equal", origin="lower", alpha=0.85, zorder=0)
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_aspect("equal")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("Subtask 2.2: Swarm (IVP + RK4, walls = repulsion like robots)")

    ax.fill(np.r_[x_left, x_right[::-1]], np.r_[y_left, y_right[::-1]],
            color="lightgray", edgecolor="gray", linewidth=1, alpha=0.7, zorder=1)
    ax.plot(x_c, y_c, "k-", lw=1.5, label="Path centerline", zorder=2)
    ax.plot(A[0], A[1], "go", markersize=12, label="A", zorder=3)
    ax.plot(B[0], B[1], "rs", markersize=12, label="B", zorder=3)

    circles = []
    for r in robots:
        color = "blue" if r["goal"] is B else "red"
        circ = Circle((r["x"], r["y"]), robot_radius, color=color, ec="black", lw=1, zorder=4)
        ax.add_patch(circ)
        circles.append(circ)

    def init():
        for i, r in enumerate(robots):
            circles[i].center = (r["x"], r["y"])
        return circles

    all_reached_flag = [False]

    def frame_gen():
        for i in range(2000):
            yield i
            if all_reached_flag[0]:
                return

    def animate(frame):
        vec = state_to_vec(robots)
        for _ in range(N_SUBSTEPS):
            vec = rk4_step(vec, DT, robots, goals, x_left, y_left, x_right, y_right)
        vec_to_state(robots, vec)
        for i, r in enumerate(robots):
            circles[i].center = (r["x"], r["y"])
        if all(np.hypot(r["x"] - r["goal"][0], r["y"] - r["goal"][1]) <= GOAL_REACHED_RADIUS for r in robots):
            all_reached_flag[0] = True
        return circles

    init()

    if SAVE_VIDEO:
        output_path = OUTPUT_VIDEO
        writer = animation.PillowWriter(fps=25)
        writer.setup(fig, output_path, dpi=120)
        plt.ion()
        fig.show()
        writer.grab_frame()
        for frame in frame_gen():
            animate(frame)
            fig.canvas.draw()
            fig.canvas.flush_events()
            writer.grab_frame()
        writer.finish()
        print("Saved video to", output_path)
        plt.ioff()
    else:
        anim = animation.FuncAnimation(
            fig, animate, init_func=init, frames=frame_gen,
            interval=40, blit=True, repeat=False, cache_frame_data=False, save_count=2000
        )

    plt.show()


if __name__ == "__main__":
    main()
