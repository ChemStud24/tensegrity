import numpy as np
import torch
import tqdm

from gnn_simulator.model_predictive_control.astar_planner import norm_angle
from gnn_simulator.utilities.misc_utils import DEFAULT_DTYPE

from gnn_simulator.model_predictive_control import util_heuristic

def snap_to_grid(point, grid_step):
    snapped_pt = [round(round(point[i] / grid_step[i]) * grid_step[i], 10) for i in range(len(point))]
    return snapped_pt

def snap_to_grid_torch(pt, grid_step, boundaries):
    pt = pt.clone()
    pt[:, 0] = torch.round((pt[:, 0] - boundaries[0]) / grid_step[0])
    pt[:, 1] = torch.round(-(pt[:, 1] - boundaries[3]) / grid_step[1])
    return pt.to(torch.int)


def unsnap_to_grid_torch(idx, grid_step, boundaries):
    px = idx[0] * grid_step[0] + boundaries[0]
    py = boundaries[3] - idx[1] * grid_step[1]

    return px, py


def snap_to_grid_torch_se2(pt, stepsizes, boundaries):
    pt = pt.clone()
    pt[:, 0] = torch.round((pt[:, 0] - boundaries[0]) / stepsizes[0])
    pt[:, 1] = torch.round(-(pt[:, 1] - boundaries[3]) / stepsizes[1])
    pt[:, 2] = torch.round((norm_angle(pt[:, 2] + torch.pi))/ stepsizes[2])
    return pt.to(torch.int)


def unsnap_to_grid_torch_se2(idx, stepsizes, boundaries):
    px = idx[0] * stepsizes[0] + boundaries[0]
    py = boundaries[3] - idx[1] * stepsizes[1]
    ptheta = idx[2] * stepsizes[2] - torch.pi

    return px, py, ptheta


def simple_collision(point, obstacles):
    px, py = point
    for xmin, xmax, ymin, ymax in obstacles:
        if (xmin <= px <= xmax) and (ymin <= py <= ymax):
            return True
    return False


def wave_heuristic_dict_to_arr(goal, boundary, obstacles, grid_step, device='cpu', dtype=DEFAULT_DTYPE):
    h_val = util_heuristic.fill_grid(
        goal,
        boundary,
        grid_step,
        obstacles,
    )
    
    size_x = int((boundary[1] - boundary[0]) / grid_step[0]) + 1
    size_y = int((boundary[3] - boundary[2]) / grid_step[1]) + 1
    h_val_arr = np.zeros((size_x, size_y), dtype=np.float64)
    obs_val_arr = np.full((size_x, size_y), np.inf, dtype=np.float64)

    for i in np.arange(boundary[0], boundary[1] + grid_step[0], grid_step[0]):
        for j in np.arange(boundary[2], boundary[3] + grid_step[1], grid_step[1]):
            i, j = snap_to_grid((i, j), grid_step=grid_step)
            idx_x = round((i - boundary[0]) / grid_step[0])
            idx_y = round(-(j - boundary[3]) / grid_step[1])
            h_val_arr[idx_x, idx_y] = h_val[(i, j)]

            if (not simple_collision((i, j), obstacles)
                    and i not in [boundary[0], boundary[1]]
                    and j not in [boundary[2], boundary[3]]):
                dist_to_bound = min([
                    abs(i - boundary[0]),
                    abs(i - boundary[1]),
                    abs(j - boundary[2]),
                    abs(j - boundary[3])
                ])

                dist_to_obs = []
                for xmin, xmax, ymin, ymax in obstacles:
                    if xmin <= i <= xmax:
                        dist_to_obs.append(min(abs(j - ymin), abs(j - ymax)))
                    elif ymin <= j <= ymax:
                        dist_to_obs.append(min(abs(i - xmin), abs(i - xmax)))
                    else:
                        dist_to_obs.append(min([
                            np.sqrt((i - xmin) ** 2 + (j - ymin) ** 2),
                            np.sqrt((i - xmin) ** 2 + (j - ymax) ** 2),
                            np.sqrt((i - xmax) ** 2 + (j - ymin) ** 2),
                            np.sqrt((i - xmax) ** 2 + (j - ymax) ** 2),
                        ]))
                dist_to_obs = min(dist_to_obs) if dist_to_obs else np.inf
                dist = min(dist_to_obs, dist_to_bound)
                dist = max(dist, 0.0)
                penalty = 100 * np.exp(-0.3 * dist)
                # penalty = 10. * max(0, (6.5 - dist)) ** 2
                # penalty = 10. / (dist ** 2 + 1e-6)

                obs_val_arr[idx_x, idx_y] = penalty

    # plot(h_val_arr, obs_val_arr, boundary, grid_step)

    h_val_arr = torch.from_numpy(h_val_arr).to(device).to(dtype)
    obs_val_arr = torch.from_numpy(obs_val_arr).to(device).to(dtype)

    return h_val_arr, obs_val_arr


def wave_heuristic_dict_to_arr_se2(goal, gaits, boundary, obstacles, stepsizes, device='cpu', dtype=DEFAULT_DTYPE):
    _, h_val = util_heuristic.goal_rooted_motion_prim(
        goal,
        boundary,
        gaits,
        obstacles,
    )

    _, h_val = util_heuristic.goal_rooted_motion_prim_gpu(
        goal,
        boundary,
        gaits,
        obstacles,
    )

    size_x = int((boundary[1] - boundary[0]) / stepsizes[0]) + 1
    size_y = int((boundary[3] - boundary[2]) / stepsizes[1]) + 1
    size_theta = int(2 * np.pi / stepsizes[2]) + 1
    obs_val_arr = np.full((size_x, size_y), 0, dtype=np.float64)

    # Generate all grid points via meshgrid
    xs = np.arange(boundary[0], boundary[1] + stepsizes[0], stepsizes[0])
    ys = np.arange(boundary[2], boundary[3] + stepsizes[1], stepsizes[1])
    thetas = np.arange(-np.pi, np.pi, stepsizes[2])
    X, Y, T = np.meshgrid(xs, ys, thetas, indexing='ij')
    query_points = np.stack([X.ravel(), Y.ravel(), T.ravel()], axis=1)

    # Snap to grid (correct floating-point drift)
    for dim in range(3):
        query_points[:, dim] = np.round(
            np.round(query_points[:, dim] / stepsizes[dim]) * stepsizes[dim], 10
        )

    # Compute array indices
    idx_x = np.round((query_points[:, 0] - boundary[0]) / stepsizes[0]).astype(int)
    idx_y = np.round(-(query_points[:, 1] - boundary[3]) / stepsizes[1]).astype(int)
    idx_theta = np.round((query_points[:, 2] + np.pi) / stepsizes[2]).astype(int)

    # Batch query all points at once
    if hasattr(h_val, 'batch_query'):
        costs = h_val.batch_query(query_points)
    else:
        costs = np.array([h_val(tuple(p)) for p in tqdm.tqdm(query_points)])

    h_val_arr = np.zeros((size_x, size_y, size_theta), dtype=np.float64)
    h_val_arr[idx_x, idx_y, idx_theta] = costs

    h_val_arr = torch.from_numpy(h_val_arr).to(device).to(dtype)
    obs_val_arr = torch.from_numpy(obs_val_arr).to(device).to(dtype)

    return h_val_arr, obs_val_arr


def heuristic_dir_old(cost_grid, snapped_pt, num_steps=20):
    # snapped_x, snapped_y = snapped_pt.cpu().flatten().numpy().tolist()
    snapped_x, snapped_y = snapped_pt
    gaits = [
        [1, 0], [0, 1], [0, -1], [-1, 0],
        [1, 1], [-1, 1], [1, -1], [-1, -1]
    ]

    open_list = [(snapped_x, snapped_y)]
    for n in range(num_steps):
        new_open_list = set()
        for x, y in open_list:
            for g in gaits:
                x_, y_ = x + g[0], y + g[1]
                if 0 <= x_ < cost_grid.shape[0] and 0 <= y_ < cost_grid.shape[1]:
                    new_open_list.add((x_, y_))
        open_list = new_open_list

    best_score, best_pt = 1e9, [0, 0]
    for x, y in open_list:
        score = cost_grid[x, y].item()
        if score < best_score:
            best_score = score
            best_pt = [x, y]

    return best_pt


def heuristic_dir_r2(cost_grid, snapped_pt, radius=20):
    if isinstance(snapped_pt, torch.Tensor):
        snapped_pt = snapped_pt.cpu().flatten().numpy().tolist()
    snapped_x, snapped_y = snapped_pt

    if isinstance(cost_grid, np.ndarray):
        cost_grid = torch.from_numpy(cost_grid)

    w, h = cost_grid.shape
    xmin = max(snapped_x - radius, 0)
    xmax = min(snapped_x + radius + 1, w)
    ymin = max(snapped_y - radius, 0)
    ymax = min(snapped_y + radius + 1, h)

    subgrid = cost_grid[xmin:xmax, ymin:ymax]
    flat_idx = torch.argmin(subgrid)
    best_pt = torch.unravel_index(flat_idx, subgrid.shape)
    best_pt = (best_pt[0] + xmin, best_pt[1] + ymin)

    return best_pt


def heuristic_dir_se2(cost_grid, snapped_pt, radius=20):
    if isinstance(snapped_pt, torch.Tensor):
        snapped_pt = snapped_pt.cpu().flatten().numpy().tolist()
    snapped_x, snapped_y, snapped_theta = snapped_pt

    if isinstance(cost_grid, np.ndarray):
        cost_grid = torch.from_numpy(cost_grid)

    w, h, d = cost_grid.shape
    xmin = max(snapped_x - radius, 0)
    xmax = min(snapped_x + radius + 1, w)
    ymin = max(snapped_y - radius, 0)
    ymax = min(snapped_y + radius + 1, h)
    thetamin = max(snapped_theta - radius, 0)
    thetamax = min(snapped_theta + radius + 1, d)

    subgrid = cost_grid[xmin:xmax, ymin:ymax, thetamin:thetamax]
    flat_idx = torch.argmin(subgrid)
    best_pt = torch.unravel_index(flat_idx, subgrid.shape)
    best_pt = (best_pt[0] + xmin, best_pt[1] + ymin, best_pt[2] + thetamin)

    return best_pt


def plot(h_val_arr, obs_val_arr, boundary, grid_step):
    import matplotlib.pyplot as plt

    plt.imshow(h_val_arr.T)
    plt.colorbar()
    plt.title("dist")
    plt.show()

    obs_val_arr_cpy = obs_val_arr.copy()
    # obs_val_arr_cpy[obs_val_arr_cpy == np.inf] = -1.
    # obs_val_arr_cpy[obs_val_arr_cpy == -1] = obs_val_arr_cpy.max() + 10
    obs_val_arr_cpy2 = np.clip(obs_val_arr_cpy, 0., 500)

    x = np.linspace(boundary[1], boundary[0], h_val_arr.shape[1])
    y = np.linspace(boundary[2], boundary[3], h_val_arr.shape[0])
    X, Y = np.meshgrid(x, y)

    plt.imshow(obs_val_arr_cpy2.T)
    plt.colorbar()
    plt.title("obs")
    plt.show()
    plt.imshow((obs_val_arr_cpy2 + h_val_arr).T)
    plt.colorbar()
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # surf = ax.plot_surface(X, Y, (obs_val_arr_cpy + h_val_arr).T)
    plt.title("combined")
    plt.show()

    h_val_arr_cpy = h_val_arr.copy()

    cost_grid = h_val_arr_cpy
    u, v = np.zeros_like(h_val_arr_cpy), np.zeros_like(h_val_arr_cpy)
    for i in range(cost_grid.shape[0]):
        if i % 10 != 0:
            continue
        # print(i)
        for j in range(cost_grid.shape[1]):
            if j % 10 != 0:
                continue
            if cost_grid[i, j] == np.inf:
                continue

            ii, jj = unsnap_to_grid_torch((i, j), grid_step=grid_step, boundaries=boundary)
            best_pt = heuristic_dir_r2(cost_grid, [i, j], 1)
            best_pt_ = unsnap_to_grid_torch(best_pt, grid_step=grid_step, boundaries=boundary)
            uu, vv = best_pt_[0] - ii, best_pt_[1] - jj

            print((i, j), best_pt, (ii, jj), best_pt_, (uu, vv))

            u[i, j] = uu
            v[i, j] = vv

    plt.figure()
    skip = 10
    plt.quiver(Y[::skip, ::skip], X[::skip, ::skip], u[::skip, ::skip], v[::skip, ::skip])
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Vector Field")
    # plt.axis('equal')
    plt.show()
