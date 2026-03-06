import heapq
import time

import numpy as np
import random
import multiprocessing as mp

import torch

from gnn_simulator.model_predictive_control.util_heuristic import dist_heuristic, l2_dist, coll_det, fill_grid, snap_to_grid

from scipy.spatial import KDTree

from gnn_simulator.utilities import torch_quaternion


###DISCLAIMER
# I know that this has a problem with not recognizing angles that wrap around
# But for this purpose it is fine
# Plus I can't figure out how to handle this without looping thorugh all the points which would ruin the whole point of using a kd-tree

def is_point_within_distance(point, closed_list, distance):
    """
    Check if a point is within a certain distance of any point in the dictionary.

    Parameters:
    - point: tuple (x, y) representing the query point.
    - points_dict: dictionary {key: (x, y)} representing other points.
    - distance: float, the maximum distance to check.

    Returns:
    - bool: True if the point is within the distance of any dictionary point, False otherwise.
    """
    # Extract the points from the dictionary
    # points = list(closed_list)

    # Build a KDTree
    # print(closed_list)
    tree = KDTree(closed_list)

    # Query the KDTree
    indices = tree.query_ball_point(point, distance)

    return len(indices) > 0


def rel_mov(x, y, theta):
    # Calculate the change in x and y using trigonometry
    dx = x * np.cos(theta) - y * np.sin(theta)
    dy = x * np.sin(theta) + y * np.cos(theta)
    return dx, dy


# def angle_norm(x):
#     return x % (2 * np.pi)

def norm_angle(new_angle):
    return (new_angle + np.pi) % (2 * np.pi) - np.pi

def heuristic(a, b, obstacles, heur_type, grid_step, k=0, grid=[]):
    if heur_type == "wave":
        approx = snap_to_grid(a[:2], grid_step)
        if approx in grid:
            return grid[approx]
        else:
            return np.inf
    else:
        return dist_heuristic(a[:2], b[:2], obstacles, k)


# A* algorithm implementation
def astar(start,
          goal,
          gaits,
          obstacles=(),
          tolerance=0.1,
          rot_tol=np.pi / 4,
          repeat_tol=0.07,
          single_push=False,
          stochastic=True,
          heur_type="dist",
          boundary=(-1, 1, -1, 1),
          grid_step=0.01,
          robot_dims=(2.95, 1.5),
          wave_h=None):
    if heur_type == "wave":
        h = fill_grid(goal[:2], boundary, grid_step, obstacles=obstacles) if wave_h is None else wave_h
    else:
        h = {}
    # Initialize open and closed lists
    open_list = []
    heapq.heappush(open_list, (0, start, -1))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal, obstacles, heur_type, grid_step, grid=h)}
    closed_list = []  # only used to check if already visited using kd tree

    while open_list:
        # Get the node with the lowest f_score value
        node = heapq.heappop(open_list)
        current = node[1]

        if (coll_det(current, obstacles, boundary=boundary, robot_dims=robot_dims)
                or (len(closed_list) > 0 and is_point_within_distance(current[:2], closed_list, repeat_tol))):
            closed_list.append(current[:2])
            continue

        if not single_push:
            closed_list.append(current[:2])

        # If the goal is reached, reconstruct and return the path
        if (l2_dist(current[:2], goal[:2]) <= tolerance
                and min(
                    abs(norm_angle(current[2]) - goal[2]),
                    np.pi - abs(norm_angle(current[2]) - goal[2])
                ) <= rot_tol):
            path = []
            movements = []
            while current in came_from:
                prev = came_from[current][0]
                move = came_from[current][1]
                path.append(current)
                movements.append(move)
                current = prev
            path.append(start)
            return path[::-1], movements[::-1], h

        if single_push:
            if stochastic:
                each_neighbor = []
            else:
                best_neighbor = None
        # best_f = 0
        for k in range(len(gaits)):
            gait_num = gaits[k]
            dx, dy = rel_mov(gait_num[0], gait_num[1], current[2])
            neighbor = (current[0] + dx, current[1] + dy, norm_angle(current[2] + gait_num[2]))

            tentative_g_score = g_score[current] + np.sqrt(dx ** 2 + dy ** 2)
            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = (current, k)
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor[:2], goal, obstacles, heur_type, grid_step,
                                                                  grid=h)
                if single_push:
                    if stochastic:
                        each_neighbor.append((f_score[neighbor], neighbor, k))
                    else:
                        if best_neighbor == None or f_score[neighbor] < best_neighbor[0]:
                            best_neighbor = (f_score[neighbor], neighbor, k)
                else:
                    heapq.heappush(open_list, (f_score[neighbor], neighbor, k))

        if single_push:
            if stochastic:
                if len(each_neighbor) == 0:
                    closed_list.append(current[:2])
                    continue
                elif len(each_neighbor) == 1:
                    heapq.heappush(open_list, each_neighbor[0])
                beta = 1.0
                weights = [np.exp(-beta * node[0]) for node in each_neighbor]
                total_weight = sum(weights)
                probabilities = [weight / total_weight for weight in weights]

                # Select a node based on the probabilities
                best_neighbor = random.choices(each_neighbor, weights=probabilities, k=1)[0]
                heapq.heappush(open_list, best_neighbor)
                heapq.heappush(open_list, node)
            else:
                if best_neighbor != None:
                    heapq.heappush(open_list, best_neighbor)
                    heapq.heappush(open_list, node)
                else:
                    closed_list.append(current[:2])

    # Return empty path if no path found
    print("Can't Find Path")
    return [], [], h


def astar_w_mp_timeout(astar_kwargs, timeout=2):
    queue = mp.Queue()
    process = mp.Process(target=mp_astar, args=(queue, astar_kwargs))

    start_time = time.time()

    process.start()
    process.join(timeout)

    while True:
        path, movements = None, []
        if not queue.empty():
            path, movements, _ = queue.get()
            break
        elif (time.time() - start_time) > timeout:
            print(f"A* exceeded timelimit {timeout} seconds.")
            process.terminate()
            break

    # path, movements = None, []
    # if process.is_alive():
    #     print(f"A* exceeded timelimit {timeout} seconds.")
    #     process.terminate()
    # elif not queue.empty():
    #     path, movements, _ = queue.get()

    return path, movements


def mp_astar(queue, astar_kwargs):
    output = astar(**astar_kwargs)
    queue.put(output)

    return queue


class TensegrityAStarPlanner(torch.nn.Module):

    def __init__(self,
                 gaits,
                 gait_deltas,
                 obstacles,
                 boundary,
                 tol=1.0,
                 goal=None,
                 rot_tol=2 * np.pi,
                 repeat_tol=0.01,
                 single_push=False,
                 stochastic=True,
                 heur_type="dist",
                 grid_step=0.1,
                 robot_rod_length=2.95,
                 robot_dims=(2.95, 1.5)):
        super().__init__()

        # self.curr_pose = curr_pose
        self.robot_rod_length = robot_rod_length
        self.robot_dims = robot_dims

        self.gaits = gaits
        self.gait_deltas = gait_deltas
        self.goal = goal
        self.obstacles = obstacles
        self.boundary = boundary
        self.tol = tol
        self.rot_tol = rot_tol
        self.repeat_tol = repeat_tol
        self.single_push = single_push
        self.stochastic = stochastic
        self.heur_type = heur_type
        self.grid_step = grid_step
        self.end_pts = None

        self.path = None
        self.movements = None

        self.wave_h = None
        if heur_type == 'wave':
            self.wave_h = fill_grid(
                self.goal[:2],
                self.boundary,
                self.grid_step,
                obstacles=self.obstacles
            )

    def reset_sim_state(self, curr_state, motor_speeds, rest_lengths, batch_size=1):
        pass

    def state_to_end_pts(self, state):
        state_ = state.reshape(-1, 13, 1)
        pos = state_[:, :3]
        quat = state_[:, 3:]
        prins = torch_quaternion.compute_prin_axis(torch.from_numpy(quat)).numpy()

        half_len = self.robot_rod_length / 2
        end_pts0 = pos - half_len * prins
        end_pts1 = pos + half_len * prins

        end_pts = np.hstack((end_pts0, end_pts1)).reshape(-1, 3)
        return end_pts

    @staticmethod
    def end_pts_to_pose(end_pts):
        end_pts = end_pts.reshape(-1, 3, 1)
        pos = (end_pts[::2] + end_pts[1::2]) / 2
        prin = end_pts[1::2] - end_pts[::2]
        prin /= np.linalg.norm(prin, axis=1, keepdims=True)
        quat = torch_quaternion.compute_quat_btwn_z_and_vec(torch.from_numpy(prin)).numpy()

        pose = np.hstack((pos, quat)).reshape(-1, 7)
        return pose

    def dist_costs(self, curr_state):
        # curr_state = self.map(curr_state)
        curr_state_ = curr_state.reshape(-1, 13)
        com = curr_state_[:, :3].mean(axis=0)

        dist_costs = ((com[0] - self.goal[0]) ** 2 + (com[1] - self.goal[1]) ** 2) ** 0.5

        return dist_costs.flatten()

    def set_goals(self, goal):
        self.goal = goal

    def pose_to_se2(self, pose):
        pose = pose.reshape(-1, 7, 1)
        com = pose[:, :2].mean(axis=0, keepdims=True)

        prin = torch_quaternion.compute_prin_axis(
            torch.from_numpy(pose[:, 3:])
        ).numpy().mean(axis=0, keepdims=True)[:, :2]
        prin /= np.linalg.norm(prin, axis=1, keepdims=True)
        angle = np.arctan2(prin[:, 1:], prin[:, :1])

        se2 = np.hstack([com, angle])

        return se2

    def plan(self, prev_n_pose_time_tups, curr_rest_lens, curr_motor_speeds):
        assert self.goal is not None

        curr_pose, curr_timestamp = prev_n_pose_time_tups[-1]
        start = tuple(self.pose_to_se2(curr_pose).flatten().tolist())
        # print(start)

        kwargs = dict(
            start=start,
            goal=self.goal,
            gaits=self.gait_deltas,
            obstacles=self.obstacles,
            tolerance=self.tol,
            rot_tol=self.rot_tol,
            repeat_tol=self.repeat_tol,
            single_push=self.single_push,
            stochastic=self.stochastic,
            heur_type=self.heur_type,
            boundary=self.boundary,
            grid_step=self.grid_step,
            wave_h=self.wave_h,
        )
        path, tmp_movements = astar_w_mp_timeout(kwargs, 3)
        # print(path[1])

        # Return empty path if no path found
        if len(tmp_movements) > 0:
            self.movements = tmp_movements[1:]
            self.path = path[1:]
            move = self.gaits[tmp_movements[0]]
            return move, self.path
        elif self.movements is not None and len(self.movements) > 0:
            print("Can't find path, returning next move from previous plan")
            move = self.gaits[self.movements[0]]
            self.path, self.movements = self.path[1:], self.movements[1:]
            return move, self.path
        else:
            print("Can't find path, no previous plan available, returning random move")
            move = random.choice(self.gaits)
            path = []
            return move, path
