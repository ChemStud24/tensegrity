import heapq
import math

import numpy as np
import tqdm
from scipy.spatial import KDTree


def get_rotated_corners(x, y, w, h, theta):
    dx = w / 2
    dy = h / 2

    corners = [
        (-dx, -dy),
        (dx, -dy),
        (dx, dy),
        (-dx, dy)
    ]

    rotated_corners = []
    for corner in corners:
        cx = corner[0] * np.cos(theta) - corner[1] * np.sin(theta)
        cy = corner[0] * np.sin(theta) + corner[1] * np.cos(theta)
        rotated_corners.append((x + cx, y + cy))

    return rotated_corners


def check_overlap(corners1, corners2):
    # Use Separating Axis Theorem (SAT) to check for overlap
    axes = get_axes(corners1) + get_axes(corners2)

    for axis in axes:
        if not projection_overlap(corners1, corners2, axis):
            return False
    return True


def get_axes(corners):
    axes = []
    for i in range(len(corners)):
        p1 = corners[i]
        p2 = corners[(i + 1) % len(corners)]
        edge = (p2[0] - p1[0], p2[1] - p1[1])
        axis = (-edge[1], edge[0])  # Perpendicular vector
        length = np.sqrt(axis[0] ** 2 + axis[1] ** 2)
        axes.append((axis[0] / length, axis[1] / length))
    return axes


def projection_overlap(corners1, corners2, axis):
    def project(corners, axis):
        return [corner[0] * axis[0] + corner[1] * axis[1] for corner in corners]

    p1 = project(corners1, axis)
    p2 = project(corners2, axis)

    if max(p1) < min(p2) or max(p2) < min(p1):
        return False
    return True


def coll_det(point, obstacles, robot_dims=(0.33, 0.15), boundary=(-3, 1, -1.4, 0.4)):
    # (0.4, 0.27)
    # (-1.0, 1.0, -1.0, 4.0)):
    # boundary is (x_min, x_max, y_min, y_max)
    robot_x, robot_y, robot_theta = point
    robot_w, robot_h = robot_dims

    # Calculate robot bounding box corners
    robot_corners = get_rotated_corners(robot_x, robot_y, robot_w, robot_h, robot_theta)
    # print(robot_corners)

    for corner in robot_corners:
        if boundary[0] > corner[0] or boundary[1] < corner[0] or boundary[2] > corner[1] or boundary[3] < corner[1]:
            # print("Border", str(corner))
            # print("Border")
            return True

    for obs_x_min, obs_x_max, obs_y_min, obs_y_max in obstacles:
        obs_x = (obs_x_max + obs_x_min) / 2
        obs_y = (obs_y_max + obs_y_min) / 2
        obs_w = obs_x_max - obs_x_min
        obs_h = obs_y_max - obs_y_min

        # Calculate obstacle bounding box corners (assuming no rotation)
        obstacle_corners = get_rotated_corners(obs_x, obs_y, obs_w, obs_h, 0)

        # Check if there is overlap between the robot and the obstacle
        if check_overlap(robot_corners, obstacle_corners):
            # print("obstacle")
            return True

    return False


def reverse_gait(gait):
    reversed_gait = []

    for step in gait:
        s = [-step[0], -step[1], -step[2]]
        # s = [-step[0], -step[1], (step[2] + np.pi) % (2.0 * np.pi)]

        reversed_gait.append(s)

    return reversed_gait


def l2_dist(a, b):
    return np.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def dist_heuristic(a, b, obstacles=[], k=0):
    dist = l2_dist(a, b)
    if k == 0 or len(obstacles) < 1:
        return dist

    penalty = 0
    for obstacle in obstacles:
        distance_to_obstacle = l2_dist(a, obstacle)
        penalty += 1 / (1 + distance_to_obstacle)

    return dist + k * penalty


# print(dist_heuristic((1.5982203824585355, -0.38906994686732577),(0.0,3.0)))

class HeuristicQueryBatch:
    """Batch-capable heuristic query using KDTree for fast vectorized lookups."""

    def __init__(self, tree, states, kdtree):
        self.tree = tree
        self.states = states  # numpy array (N, 3)
        self.kdtree = kdtree
        self.costs = np.array([tree[tuple(s)]['cost'] for s in states])

    @staticmethod
    def _normalize_angle(theta):
        return (theta + np.pi) % (2 * np.pi) - np.pi

    def __call__(self, query_state, k=10):
        """Single-point query (backward compatible)."""
        qx, qy, qtheta = query_state
        distances, indices = self.kdtree.query([qx, qy], k=min(k, len(self.states)))

        candidates = []
        for idx in indices.flatten():
            candidate = tuple(self.states[idx])
            cx, cy, ctheta = candidate
            spatial_dist = np.sqrt((qx - cx) ** 2 + (qy - cy) ** 2)
            angular_dist = abs(self._normalize_angle(qtheta - ctheta))
            se2_dist = spatial_dist + 0.5 * angular_dist
            connection_cost = spatial_dist
            path_cost_to_goal = self.tree[candidate]['cost']
            total_cost = connection_cost + path_cost_to_goal
            candidates.append((se2_dist, total_cost))

        if len(candidates) == 0:
            return None

        candidates.sort(key=lambda x: x[0])
        top_3 = candidates[:min(3, len(candidates))]
        return np.mean([cost for _, cost in top_3])

    def batch_query(self, query_points, k=10):
        """
        Vectorized batch query for all grid points at once.

        Args:
            query_points: (M, 3) array of (x, y, theta) points
            k: number of nearest neighbors to consider

        Returns:
            (M,) array of costs
        """
        k = min(k, len(self.states))
        # Batch KDTree query on XY only: (M, k)
        distances, indices = self.kdtree.query(query_points[:, :2], k=k)

        # Ensure 2D even if k=1
        if k == 1:
            distances = distances[:, np.newaxis]
            indices = indices[:, np.newaxis]

        # Spatial distances from KDTree: (M, k)
        spatial_dists = distances

        # Angular distances: (M, k)
        query_theta = query_points[:, 2:3]  # (M, 1)
        neighbor_theta = self.states[indices, 2]  # (M, k)
        angular_dists = np.abs(self._normalize_angle(query_theta - neighbor_theta))

        # SE2 distance: (M, k)
        se2_dists = spatial_dists + 0.5 * angular_dists

        # Total cost = spatial_dist (connection) + path_cost_to_goal: (M, k)
        path_costs = self.costs[indices]
        total_costs = spatial_dists + path_costs

        # Sort by SE2 distance, take top 3, average
        top_n = min(3, k)
        sort_idx = np.argsort(se2_dists, axis=1)[:, :top_n]  # (M, top_n)
        top_costs = np.take_along_axis(total_costs, sort_idx, axis=1)  # (M, top_n)

        return top_costs.mean(axis=1)  # (M,)


########
def snap_to_grid(point, grid_step):
    snapped_x = round(round(point[0] / grid_step[0]) * grid_step[0], 10)
    snapped_y = round(round(point[1] / grid_step[1]) * grid_step[1], 10)
    return (snapped_x, snapped_y)


def simple_collision(point, obstacles):
    px, py = point
    for obs_x_min, obs_x_max, obs_y_min, obs_y_max in obstacles:
        ox, oy = (obs_x_max + obs_x_min) / 2, (obs_y_max + obs_y_min) / 2
        half_w, half_h = (obs_x_max - obs_x_min) / 2, (obs_y_max - obs_y_min) / 2
        if (ox - half_w <= px <= ox + half_w) and (oy - half_h <= py <= oy + half_h):
            return True
    return False


def fill_grid(goal, boundary, grid_step=[0.1, 0.1], obstacles=()):
    # BFS from goal
    h_val = {}
    unassigned = set()
    obs_loc = set()
    goal = snap_to_grid(goal, grid_step)
    # h_val[goal]=0
    gaits = [[grid_step[0], 0], [0, grid_step[1]], [0, -grid_step[1]], [-grid_step[0], 0], \
             [grid_step[0], grid_step[1]], [-grid_step[0], grid_step[1]], [grid_step[0], -grid_step[1]], [-grid_step[0], -grid_step[1]]]

    for i in np.arange(boundary[0], boundary[1] + grid_step[0], grid_step[0]):
        for j in np.arange(boundary[2], boundary[3] + grid_step[1], grid_step[1]):
            current = snap_to_grid((i, j), grid_step=grid_step)
            if simple_collision((i, j), obstacles):
                h_val[current] = np.inf
                obs_loc.add(current)
                # all_points.remove()
            else:
                unassigned.add(current)

    open_list = []
    heapq.heappush(open_list, (0, goal))

    while unassigned:
        if not open_list:
            break
        node = heapq.heappop(open_list)

        current = snap_to_grid(node[1], grid_step)
        if current in obs_loc or current not in unassigned:
            continue

        h_val[current] = node[0]
        unassigned.remove(current)

        for k in range(len(gaits)):
            gait_num = gaits[k]
            neighbor = (current[0] + gait_num[0], current[1] + gait_num[1])

            if k < 4:
                cost = 1
            else:
                cost = np.sqrt(2)

            penalty = compute_xy_obs_cost(boundary, neighbor, obstacles)

            cost += penalty

            heapq.heappush(open_list, (node[0] + cost, neighbor))
    return h_val


def compute_xy_obs_cost(boundary, neighbor, obstacles):
    if (not simple_collision((neighbor[0], neighbor[1]), obstacles)
            and neighbor[0] not in [boundary[0], boundary[1]]
            and neighbor[1] not in [boundary[2], boundary[3]]):
        dist_to_bound = min([
            abs(neighbor[0] - boundary[0]),
            abs(neighbor[0] - boundary[1]),
            abs(neighbor[1] - boundary[2]),
            abs(neighbor[1] - boundary[3])
        ])

        dist_to_obs = []
        for xmin, xmax, ymin, ymax in obstacles:
            if xmin <= neighbor[0] <= xmax:
                dist_to_obs.append(min(abs(neighbor[1] - ymin), abs(neighbor[1] - ymax)))
            elif ymin <= neighbor[1] <= ymax:
                dist_to_obs.append(min(abs(neighbor[0] - xmin), abs(neighbor[0] - xmax)))
            else:
                dist_to_obs.append(min([
                    np.sqrt((neighbor[0] - xmin) ** 2 + (neighbor[1] - ymin) ** 2),
                    np.sqrt((neighbor[0] - xmin) ** 2 + (neighbor[1] - ymax) ** 2),
                    np.sqrt((neighbor[0] - xmax) ** 2 + (neighbor[1] - ymin) ** 2),
                    np.sqrt((neighbor[0] - xmax) ** 2 + (neighbor[1] - ymax) ** 2),
                ]))
        dist_to_obs = min(dist_to_obs) if dist_to_obs else np.inf
        dist = max(min(dist_to_obs, dist_to_bound), 0)
        penalty = 20 * np.exp(-0.7 * dist)
    else:
        penalty = np.inf
    return penalty


def voxel_coverage_ratio(points, boundary, stepsizes):
    """
    Compute the percentage of voxels in a 3D domain that contain at least one point.

    Args:
        points: List of (x, y, theta) tuples or (N, 3) array.
        boundary: (x_min, x_max, y_min, y_max, theta_min, theta_max).
        stepsizes: (x_step, y_step, theta_step).

    Returns:
        float: Percentage (0-100) of voxels with at least 1 point. Returns 0.0 if
               the domain has no voxels (e.g., invalid stepsizes).

    Note:
        Theta is periodic: values are wrapped into [theta_min, theta_max) before
        voxel assignment.
    """
    pts = np.asarray(points, dtype=float)
    if pts.ndim == 1:
        pts = pts.reshape(1, -1)
    x, y, theta = pts[:, 0], pts[:, 1], pts[:, 2]

    x_min, x_max, y_min, y_max = boundary
    theta_min, theta_max = -np.pi, np.pi
    x_step, y_step, theta_step = stepsizes

    t = theta_min + np.mod(theta - theta_min, 2 * np.pi)

    nx = int(np.ceil((x_max - x_min) / x_step))
    ny = int(np.ceil((y_max - y_min) / y_step))
    ntheta = int(np.ceil(2 * np.pi / theta_step))

    mask = (x_min <= x) & (x <= x_max) & (y_min <= y) & (y <= y_max) & (theta_min <= t) & (t <= theta_max)

    i = np.clip(np.floor((x[mask] - x_min) / x_step).astype(int), 0, nx - 1)
    j = np.clip(np.floor((y[mask] - y_min) / y_step).astype(int), 0, ny - 1)
    k = np.clip(np.floor((t[mask] - theta_min) / theta_step).astype(int), 0, ntheta - 1)

    linear_idx = i * (ny * ntheta) + j * ntheta + k
    occupied_count = np.unique(linear_idx).size

    total_voxels = nx * ny * ntheta
    if total_voxels <= 0:
        return 0.0

    return 100.0 * occupied_count / total_voxels


def wave_heuristic(start, goal, grid_step=0.1, obstacles=()):
    start = snap_to_grid(start, grid_step)  # now in the format (x,y)
    goal = snap_to_grid(goal, grid_step)

    if simple_collision(start, obstacles):
        return 1000

    gaits = [[grid_step, 0], [0, grid_step], [0, -grid_step], [-grid_step, 0], \
             [grid_step, grid_step], [-grid_step, grid_step], [grid_step, -grid_step], [-grid_step, -grid_step]]

    open_list = []

    # came_from = {}
    g_score = {start: 0}
    f_score = {start: dist_heuristic(start, goal)}
    heapq.heappush(open_list, (f_score[start], start))
    closed_list = set()  # only used to check if already visited using kd tree

    while open_list:
        # Get the node with the lowest f_score value
        node = heapq.heappop(open_list)
        current = node[1]
        closed_list.add(current)

        if simple_collision(current, obstacles):
            continue

        # If the goal is reached, reconstruct and return the path
        if current == goal:
            return g_score[current]

        for k in range(len(gaits)):
            gait_num = gaits[k]
            neighbor = snap_to_grid((current[0] + gait_num[0], current[1] + gait_num[1]), grid_step)

            if neighbor in closed_list:
                continue

            if k < 4:
                cost = 1
            else:
                cost = np.sqrt(2)

            tentative_g_score = g_score[current] + cost
            if tentative_g_score < g_score.get(neighbor, float('inf')):
                # came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + dist_heuristic(neighbor[:2], goal, obstacles)
                heapq.heappush(open_list, (f_score[neighbor], neighbor))


def goal_rooted_motion_prim(goal, boundary, gaits, obstacles=[], robot_dims=(2.95, 1.5), repeat_tol=0.07):
    # A* from goal using reverse gaits
    gaits = reverse_gait(gaits)

    _pi = math.pi
    _2pi = 2.0 * _pi

    # --- Precomputation ---

    # Obstacle corners for SAT (axis-aligned, theta=0)
    precomputed_obs = []
    for obs_x_min, obs_x_max, obs_y_min, obs_y_max in obstacles:
        ox = (obs_x_max + obs_x_min) * 0.5
        oy = (obs_y_max + obs_y_min) * 0.5
        hdx = (obs_x_max - obs_x_min) * 0.5
        hdy = (obs_y_max - obs_y_min) * 0.5
        corners = [(ox - hdx, oy - hdy), (ox + hdx, oy - hdy),
                    (ox + hdx, oy + hdy), (ox - hdx, oy + hdy)]
        precomputed_obs.append((obs_x_min, obs_x_max, obs_y_min, obs_y_max, corners))

    # Robot half-dims and raw (unrotated) corners
    robot_w, robot_h = robot_dims
    rhdx, rhdy = robot_w * 0.5, robot_h * 0.5
    robot_raw = [(-rhdx, -rhdy), (rhdx, -rhdy), (rhdx, rhdy), (-rhdx, rhdy)]

    # Gait costs (constant per gait)
    gait_costs = [math.sqrt(dx * dx + dy * dy) + 0.1 * abs(dtheta)
                  for dx, dy, dtheta in gaits]

    b0, b1, b2, b3 = boundary

    # --- Fast helpers ---

    def normalize_angle(theta):
        return (theta + _pi) % _2pi - _pi

    def coll_det_fast(rx, ry, cos_t, sin_t):
        """Collision detection with precomputed obstacle corners and AABB fast-path."""
        rc = [(rx + c0 * cos_t - c1 * sin_t,
               ry + c0 * sin_t + c1 * cos_t) for c0, c1 in robot_raw]

        for cx, cy in rc:
            if cx < b0 or cx > b1 or cy < b2 or cy > b3:
                return True

        if not precomputed_obs:
            return False

        rc0x, rc0y = rc[0]; rc1x, rc1y = rc[1]
        rc2x, rc2y = rc[2]; rc3x, rc3y = rc[3]
        r_xmin = min(rc0x, rc1x, rc2x, rc3x)
        r_xmax = max(rc0x, rc1x, rc2x, rc3x)
        r_ymin = min(rc0y, rc1y, rc2y, rc3y)
        r_ymax = max(rc0y, rc1y, rc2y, rc3y)

        # 2 unique robot edge normals for SAT
        n0x = -(rc1y - rc0y); n0y = rc1x - rc0x
        l0 = math.sqrt(n0x * n0x + n0y * n0y)
        n0x /= l0; n0y /= l0
        n1x = -(rc2y - rc1y); n1y = rc2x - rc1x
        l1 = math.sqrt(n1x * n1x + n1y * n1y)
        n1x /= l1; n1y /= l1

        for o_xmin, o_xmax, o_ymin, o_ymax, oc in precomputed_obs:
            # AABB rejection (covers (1,0) and (0,1) SAT axes)
            if r_xmax < o_xmin or o_xmax < r_xmin or r_ymax < o_ymin or o_ymax < r_ymin:
                continue
            # SAT along robot normal axes only (obstacle axes handled by AABB)
            overlap = True
            for ax, ay in ((n0x, n0y), (n1x, n1y)):
                rp0 = rc0x * ax + rc0y * ay
                rp1 = rc1x * ax + rc1y * ay
                rp2 = rc2x * ax + rc2y * ay
                rp3 = rc3x * ax + rc3y * ay
                r_lo = min(rp0, rp1, rp2, rp3)
                r_hi = max(rp0, rp1, rp2, rp3)
                op0 = oc[0][0] * ax + oc[0][1] * ay
                op1 = oc[1][0] * ax + oc[1][1] * ay
                op2 = oc[2][0] * ax + oc[2][1] * ay
                op3 = oc[3][0] * ax + oc[3][1] * ay
                o_lo = min(op0, op1, op2, op3)
                o_hi = max(op0, op1, op2, op3)
                if r_hi < o_lo or o_hi < r_lo:
                    overlap = False
                    break
            if overlap:
                return True
        return False

    def obs_cost_fast(nx, ny):
        """Obstacle proximity cost using math module."""
        for o_xmin, o_xmax, o_ymin, o_ymax, _ in precomputed_obs:
            if o_xmin <= nx <= o_xmax and o_ymin <= ny <= o_ymax:
                return float('inf')
        if nx == b0 or nx == b1 or ny == b2 or ny == b3:
            return float('inf')

        dist_to_bound = min(
            abs(nx - b0), abs(nx - b1),
            abs(ny - b2), abs(ny - b3)
        )

        dist_to_obs_min = float('inf')
        for o_xmin, o_xmax, o_ymin, o_ymax, _ in precomputed_obs:
            if o_xmin <= nx <= o_xmax:
                d = min(abs(ny - o_ymin), abs(ny - o_ymax))
            elif o_ymin <= ny <= o_ymax:
                d = min(abs(nx - o_xmin), abs(nx - o_xmax))
            else:
                d = min(
                    math.hypot(nx - o_xmin, ny - o_ymin),
                    math.hypot(nx - o_xmin, ny - o_ymax),
                    math.hypot(nx - o_xmax, ny - o_ymin),
                    math.hypot(nx - o_xmax, ny - o_ymax),
                )
            if d < dist_to_obs_min:
                dist_to_obs_min = d

        dist = max(min(dist_to_obs_min, dist_to_bound), 0.0)
        return 20.0 * math.exp(-0.7 * dist)

    # --- Tree initialization ---

    tree = {}
    tree[goal] = {'parent': None, 'cost': 0, 'gait_idx': None}

    # Pre-allocate states array (avoids repeated KDTree rebuilds)
    max_iterations = 100000
    max_states = max_iterations * len(gaits) + 1
    states_arr = np.empty((max_states, 3), dtype=np.float64)
    states_arr[0] = goal
    n_states = 1

    open_set = [(0, goal)]
    closed_set = set()

    min_node_distance_sq = repeat_tol * repeat_tol
    min_angle_distance = _pi / 2

    # --- Main loop ---

    progress_bar = tqdm.tqdm(total=max_iterations, desc="Building Motion Heuristic")
    iteration = 0
    while open_set and iteration < max_iterations:
        progress_bar.update(1)
        current_cost, current_state = heapq.heappop(open_set)

        if current_state in closed_set:
            continue
        closed_set.add(current_state)

        cx, cy, ctheta = current_state
        # Hoist trig outside gait loop — same cos/sin for all gaits of this node
        cos_ct = math.cos(ctheta)
        sin_ct = math.sin(ctheta)

        # Try each gait
        for gait_idx, (dx, dy, dtheta) in enumerate(gaits):
            # Apply gait in current frame
            new_x = cx + dx * cos_ct - dy * sin_ct
            new_y = cy + dx * sin_ct + dy * cos_ct
            new_theta = normalize_angle(ctheta + dtheta)

            # Fast collision detection with precomputed obstacle data
            cos_nt = math.cos(new_theta)
            sin_nt = math.sin(new_theta)
            if coll_det_fast(new_x, new_y, cos_nt, sin_nt):
                continue

            # Vectorized proximity check — no KDTree needed
            dx_arr = states_arr[:n_states, 0] - new_x
            dy_arr = states_arr[:n_states, 1] - new_y
            spatial_sq = dx_arr * dx_arr + dy_arr * dy_arr
            close_mask = spatial_sq < min_node_distance_sq
            if np.any(close_mask):
                dtheta_arr = np.abs((states_arr[:n_states, 2][close_mask] - new_theta + _pi) % _2pi - _pi)
                if np.any(dtheta_arr < min_angle_distance):
                    continue

            obs_cost = obs_cost_fast(new_x, new_y)

            new_cost = current_cost + gait_costs[gait_idx] + obs_cost

            new_state = (new_x, new_y, new_theta)
            is_new = new_state not in tree
            if is_new or tree[new_state]['cost'] > new_cost:
                tree[new_state] = {
                    'parent': current_state,
                    'cost': new_cost,
                    'gait_idx': gait_idx
                }
                heapq.heappush(open_set, (new_cost, new_state))
                if is_new:
                    states_arr[n_states] = new_state
                    n_states += 1

        iteration += 1

    progress_bar.close()

    # Build final KD-tree for queries (slice pre-allocated array to actual size)
    states = states_arr[:n_states].copy()
    kdtree = KDTree(states[:, :2])

    def query(query_state, k=10):
        """Find average path cost from query_state to goal using 3 nearest nodes"""
        qx, qy, qtheta = query_state
        distances, indices = kdtree.query([qx, qy], k=min(k, len(states)))

        candidates = []
        for idx in indices.flatten():
            candidate = tuple(states[idx])
            cx, cy, ctheta = candidate
            spatial_dist = math.sqrt((qx - cx) ** 2 + (qy - cy) ** 2)
            angular_dist = abs(normalize_angle(qtheta - ctheta))
            se2_dist = spatial_dist + 0.5 * angular_dist
            connection_cost = spatial_dist
            path_cost_to_goal = tree[candidate]['cost']
            total_cost = connection_cost + path_cost_to_goal
            candidates.append((se2_dist, total_cost))

        if len(candidates) == 0:
            return None

        candidates.sort(key=lambda x: x[0])
        top_3 = candidates[:min(3, len(candidates))]
        avg_cost = np.mean([cost for _, cost in top_3])
        return avg_cost

    batch_query = HeuristicQueryBatch(tree, states, kdtree)

    print(voxel_coverage_ratio(states, boundary, [0.1, 0.1, np.pi / 50]))

    return tree, batch_query


def goal_rooted_motion_prim_gpu(goal, boundary, gaits, obstacles=(),
                                robot_dims=(2.95, 1.5), repeat_tol=0.07,
                                device=None):
    """GPU-accelerated goal_rooted_motion_prim using PyTorch.

    Same algorithm and return type as goal_rooted_motion_prim, but batches all
    per-gait work (collision detection, proximity check, obstacle cost) on GPU.
    The priority-queue outer loop remains sequential on CPU.
    """
    import torch

    if device is None:
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    gaits = reverse_gait(gaits)
    num_gaits = len(gaits)

    _pi = math.pi
    _2pi = 2.0 * _pi

    # ---- tensors on device (created once) ---------------------------------

    gaits_t = torch.tensor(gaits, dtype=torch.float64, device=device)       # (G, 3)
    gait_dxs = gaits_t[:, 0]                                                # (G,)
    gait_dys = gaits_t[:, 1]
    gait_dthetas = gaits_t[:, 2]
    gait_costs_t = torch.sqrt(gait_dxs ** 2 + gait_dys ** 2) + 0.1 * torch.abs(gait_dthetas)

    b_t = torch.tensor(boundary, dtype=torch.float64, device=device)        # (4,)

    robot_w, robot_h = robot_dims
    rhdx, rhdy = robot_w * 0.5, robot_h * 0.5
    robot_raw_t = torch.tensor(
        [[-rhdx, -rhdy], [rhdx, -rhdy], [rhdx, rhdy], [-rhdx, rhdy]],
        dtype=torch.float64, device=device,
    )  # (4, 2)
    raw_x = robot_raw_t[:, 0]                                               # (4,)
    raw_y = robot_raw_t[:, 1]

    num_obs = len(obstacles)
    if num_obs > 0:
        obs_t = torch.tensor(obstacles, dtype=torch.float64, device=device)  # (O, 4)
        obs_cx = (obs_t[:, 0] + obs_t[:, 1]) * 0.5
        obs_cy = (obs_t[:, 2] + obs_t[:, 3]) * 0.5
        obs_hdx = (obs_t[:, 1] - obs_t[:, 0]) * 0.5
        obs_hdy = (obs_t[:, 3] - obs_t[:, 2]) * 0.5
        obs_corners_x = torch.stack([obs_cx - obs_hdx, obs_cx + obs_hdx,
                                     obs_cx + obs_hdx, obs_cx - obs_hdx], dim=1)  # (O, 4)
        obs_corners_y = torch.stack([obs_cy - obs_hdy, obs_cy - obs_hdy,
                                     obs_cy + obs_hdy, obs_cy + obs_hdy], dim=1)  # (O, 4)
    else:
        obs_t = None

    # pre-allocate states on GPU
    max_iterations = 100000
    max_states = max_iterations * num_gaits + 1
    states_gpu = torch.empty((max_states, 3), dtype=torch.float64, device=device)
    states_gpu[0] = torch.tensor(goal, dtype=torch.float64, device=device)
    n_states = 1

    min_node_distance_sq = repeat_tol * repeat_tol
    min_angle_distance = _pi / 2

    # ---- batched helpers ---------------------------------------------------

    def _batch_coll_det(xs, ys, thetas):
        """Vectorised SAT collision for G candidates.  Returns (G,) bool."""
        G = xs.shape[0]
        cos_t = torch.cos(thetas)
        sin_t = torch.sin(thetas)

        # robot corners  (G, 4)
        rc_x = xs.unsqueeze(1) + raw_x * cos_t.unsqueeze(1) - raw_y * sin_t.unsqueeze(1)
        rc_y = ys.unsqueeze(1) + raw_x * sin_t.unsqueeze(1) + raw_y * cos_t.unsqueeze(1)

        # boundary check – any corner outside ⇒ collision
        coll = ((rc_x < b_t[0]) | (rc_x > b_t[1])).any(dim=1) | \
               ((rc_y < b_t[2]) | (rc_y > b_t[3])).any(dim=1)            # (G,)

        if obs_t is None:
            return coll

        # robot AABB
        r_xmin = rc_x.min(dim=1).values
        r_xmax = rc_x.max(dim=1).values
        r_ymin = rc_y.min(dim=1).values
        r_ymax = rc_y.max(dim=1).values

        # AABB separation (covers SAT axes (1,0) and (0,1))  (G, O)
        sep = (r_xmax.unsqueeze(1) < obs_t[:, 0]) | \
              (obs_t[:, 1] < r_xmin.unsqueeze(1)) | \
              (r_ymax.unsqueeze(1) < obs_t[:, 2]) | \
              (obs_t[:, 3] < r_ymin.unsqueeze(1))

        # two robot-edge normal axes
        for i0, i1 in ((0, 1), (1, 2)):
            nx = -(rc_y[:, i1] - rc_y[:, i0])                              # (G,)
            ny =  (rc_x[:, i1] - rc_x[:, i0])
            length = torch.sqrt(nx * nx + ny * ny)
            nx = nx / length
            ny = ny / length

            # project robot corners  (G, 4) → min/max  (G,)
            rp = rc_x * nx.unsqueeze(1) + rc_y * ny.unsqueeze(1)
            r_lo = rp.min(dim=1).values
            r_hi = rp.max(dim=1).values

            # project obstacle corners  (G, O, 4)
            op = obs_corners_x.unsqueeze(0) * nx.unsqueeze(1).unsqueeze(2) + \
                 obs_corners_y.unsqueeze(0) * ny.unsqueeze(1).unsqueeze(2)
            o_lo = op.min(dim=2).values                                     # (G, O)
            o_hi = op.max(dim=2).values

            sep = sep | (r_hi.unsqueeze(1) < o_lo) | (o_hi < r_lo.unsqueeze(1))

        # overlap where NO separating axis found
        coll = coll | (~sep).any(dim=1)
        return coll

    def _batch_proximity(new_xs, new_ys, new_thetas, n):
        """(G,) bool – True if candidate is too close to an existing state."""
        if n == 0:
            return torch.zeros(new_xs.shape[0], dtype=torch.bool, device=device)

        # (G, N) pairwise squared spatial distance
        dx = states_gpu[:n, 0].unsqueeze(0) - new_xs.unsqueeze(1)
        dy = states_gpu[:n, 1].unsqueeze(0) - new_ys.unsqueeze(1)
        sp_sq = dx * dx + dy * dy

        close_sp = sp_sq < min_node_distance_sq

        # angular distance everywhere (cheap, avoids a masked scatter)
        dth = (states_gpu[:n, 2].unsqueeze(0) - new_thetas.unsqueeze(1) + _pi) % _2pi - _pi
        close_ang = torch.abs(dth) < min_angle_distance

        return (close_sp & close_ang).any(dim=1)

    def _batch_obs_cost(nxs, nys):
        """(V,) tensor of obstacle-proximity penalties."""
        V = nxs.shape[0]
        if V == 0:
            return torch.empty(0, dtype=torch.float64, device=device)

        costs = torch.full((V,), float('inf'), dtype=torch.float64, device=device)

        # inside obstacle?  (V, O)
        if obs_t is not None:
            inside = (nxs.unsqueeze(1) >= obs_t[:, 0]) & \
                     (nxs.unsqueeze(1) <= obs_t[:, 1]) & \
                     (nys.unsqueeze(1) >= obs_t[:, 2]) & \
                     (nys.unsqueeze(1) <= obs_t[:, 3])
            in_any = inside.any(dim=1)
        else:
            in_any = torch.zeros(V, dtype=torch.bool, device=device)

        on_bnd = (nxs == b_t[0]) | (nxs == b_t[1]) | (nys == b_t[2]) | (nys == b_t[3])
        invalid = in_any | on_bnd

        valid = ~invalid
        if not valid.any():
            return costs

        vnx = nxs[valid]
        vny = nys[valid]

        dist_bnd = torch.minimum(
            torch.minimum(torch.abs(vnx - b_t[0]), torch.abs(vnx - b_t[1])),
            torch.minimum(torch.abs(vny - b_t[2]), torch.abs(vny - b_t[3])),
        )

        if obs_t is not None:
            vnx_e = vnx.unsqueeze(1)                                        # (V', 1)
            vny_e = vny.unsqueeze(1)

            in_x = (vnx_e >= obs_t[:, 0]) & (vnx_e <= obs_t[:, 1])         # (V', O)
            in_y = (vny_e >= obs_t[:, 2]) & (vny_e <= obs_t[:, 3])

            d_y = torch.minimum(torch.abs(vny_e - obs_t[:, 2]),
                                torch.abs(vny_e - obs_t[:, 3]))
            d_x = torch.minimum(torch.abs(vnx_e - obs_t[:, 0]),
                                torch.abs(vnx_e - obs_t[:, 1]))
            d_corner = torch.minimum(
                torch.minimum(
                    torch.sqrt((vnx_e - obs_t[:, 0]) ** 2 + (vny_e - obs_t[:, 2]) ** 2),
                    torch.sqrt((vnx_e - obs_t[:, 0]) ** 2 + (vny_e - obs_t[:, 3]) ** 2),
                ),
                torch.minimum(
                    torch.sqrt((vnx_e - obs_t[:, 1]) ** 2 + (vny_e - obs_t[:, 2]) ** 2),
                    torch.sqrt((vnx_e - obs_t[:, 1]) ** 2 + (vny_e - obs_t[:, 3]) ** 2),
                ),
            )                                                                # (V', O)

            dist_each = torch.where(in_x, d_y, torch.where(in_y, d_x, d_corner))
            dist_obs = dist_each.min(dim=1).values                           # (V',)
        else:
            dist_obs = torch.full_like(dist_bnd, float('inf'))

        dist = torch.clamp(torch.minimum(dist_obs, dist_bnd), min=0.0)
        costs[valid] = 20.0 * torch.exp(-0.7 * dist)
        return costs

    # ---- tree / priority queue (CPU) --------------------------------------

    tree = {}
    tree[goal] = {'parent': None, 'cost': 0, 'gait_idx': None}

    open_set = [(0, goal)]
    closed_set = set()

    # ---- main loop ---------------------------------------------------------

    progress_bar = tqdm.tqdm(total=max_iterations, desc="Building Motion Heuristic (GPU)")
    iteration = 0
    while open_set and iteration < max_iterations:
        progress_bar.update(1)
        current_cost, current_state = heapq.heappop(open_set)

        if current_state in closed_set:
            continue
        closed_set.add(current_state)

        cx, cy, ctheta = current_state
        cos_ct = math.cos(ctheta)
        sin_ct = math.sin(ctheta)

        # --- compute ALL candidate states for every gait at once (GPU) ------
        new_xs     = cx + gait_dxs * cos_ct - gait_dys * sin_ct             # (G,)
        new_ys     = cy + gait_dxs * sin_ct + gait_dys * cos_ct
        new_thetas = (ctheta + gait_dthetas + _pi) % _2pi - _pi

        # --- batch collision detection (GPU) --------------------------------
        coll_mask = _batch_coll_det(new_xs, new_ys, new_thetas)              # (G,)

        # --- batch proximity check (GPU) ------------------------------------
        prox_mask = _batch_proximity(new_xs, new_ys, new_thetas, n_states)   # (G,)

        valid_mask = ~coll_mask & ~prox_mask
        if not valid_mask.any():
            iteration += 1
            continue

        # --- batch obstacle cost (GPU, valid only) --------------------------
        v_xs     = new_xs[valid_mask]
        v_ys     = new_ys[valid_mask]
        v_thetas = new_thetas[valid_mask]
        v_gcosts = gait_costs_t[valid_mask]

        obs_costs   = _batch_obs_cost(v_xs, v_ys)
        total_costs = current_cost + v_gcosts + obs_costs

        # --- single GPU → CPU sync per iteration ---------------------------
        valid_indices_cpu = valid_mask.nonzero(as_tuple=True)[0].cpu().numpy()
        v_xs_cpu     = v_xs.cpu().numpy()
        v_ys_cpu     = v_ys.cpu().numpy()
        v_thetas_cpu = v_thetas.cpu().numpy()
        costs_cpu    = total_costs.cpu().numpy()

        new_gpu_idx = []  # indices into v_* arrays for newly added states
        for i in range(len(valid_indices_cpu)):
            gi = int(valid_indices_cpu[i])
            ns = (float(v_xs_cpu[i]), float(v_ys_cpu[i]), float(v_thetas_cpu[i]))
            nc = float(costs_cpu[i])

            is_new = ns not in tree
            if is_new or tree[ns]['cost'] > nc:
                tree[ns] = {'parent': current_state, 'cost': nc, 'gait_idx': gi}
                heapq.heappush(open_set, (nc, ns))
                if is_new:
                    new_gpu_idx.append(i)

        # batch-write new states back to GPU array
        if new_gpu_idx:
            n_new = len(new_gpu_idx)
            idx = torch.tensor(new_gpu_idx, dtype=torch.long, device=device)
            states_gpu[n_states:n_states + n_new, 0] = v_xs[idx]
            states_gpu[n_states:n_states + n_new, 1] = v_ys[idx]
            states_gpu[n_states:n_states + n_new, 2] = v_thetas[idx]
            n_states += n_new

        iteration += 1

    progress_bar.close()

    # ---- build final query structures (CPU / scipy) -----------------------

    states = states_gpu[:n_states].cpu().numpy().copy()
    kdtree = KDTree(states[:, :2])

    def normalize_angle(theta):
        return (theta + _pi) % _2pi - _pi

    def query(query_state, k=10):
        """Find average path cost from query_state to goal using 3 nearest nodes"""
        qx, qy, qtheta = query_state
        distances, indices = kdtree.query([qx, qy], k=min(k, len(states)))

        candidates = []
        for idx in indices.flatten():
            candidate = tuple(states[idx])
            sx, sy, stheta = candidate
            spatial_dist = math.sqrt((qx - sx) ** 2 + (qy - sy) ** 2)
            angular_dist = abs(normalize_angle(qtheta - stheta))
            se2_dist = spatial_dist + 0.5 * angular_dist
            path_cost_to_goal = tree[candidate]['cost']
            total_cost = spatial_dist + path_cost_to_goal
            candidates.append((se2_dist, total_cost))

        if len(candidates) == 0:
            return None

        candidates.sort(key=lambda x: x[0])
        top_3 = candidates[:min(3, len(candidates))]
        return np.mean([cost for _, cost in top_3])

    batch_query = HeuristicQueryBatch(tree, states, kdtree)

    print(voxel_coverage_ratio(states, boundary, [0.1, 0.1, np.pi / 50]))

    return tree, batch_query