import json
from pathlib import Path

import numpy as np


def process_real_data(data_path,
                      config_path,
                      raw_data_path,
                      output_dir,
                      dt=0.01):
    data_arrays = load_endcap_data(data_path)
    ext_mat = get_camera_ext_mat(config_path)
    raw_json_data = load_raw_json_data(raw_data_path)

    endcaps_data_world = cam_frame_to_world_frame(data_arrays,
                                                  ext_mat)
    endcaps_data_world = shift_end_caps(endcaps_data_world)

    flatten_endcaps_world = [
        [endcaps_data_world[i][j: j + 1] for i in range(len(endcaps_data_world))]
        for j in range(endcaps_data_world[0].shape[0])
    ]
    interp_endcaps = linear_interp_seq(flatten_endcaps_world, raw_json_data, dt)

    raw_poses = compute_poses(endcaps_data_world)
    pose_json, target_gaits = combine_endcaps_and_json_data(raw_poses, endcaps_data_world, raw_json_data)

    interp_poses = compute_poses(interp_endcaps)
    interp_poses = [interp_poses[i].tolist() for i in range(len(interp_poses))]
    interp_times = [dt * i for i in range(len(interp_poses))]
    interp_endcaps = np.hstack(interp_endcaps)
    interp_endcaps = [interp_endcaps[i].reshape(-1, 3).tolist() for i in range(len(interp_endcaps))]
    interp_json = [{'time': t, 'pos': p, 'end_pts': e}
                   for p, t, e in zip(interp_poses, interp_times, interp_endcaps)]

    with Path(output_dir, 'processed_data.json').open("w") as fp:
        json.dump(pose_json, fp)

    with Path(output_dir, f'processed_data_{dt}.json').open("w") as fp:
        json.dump(interp_json, fp)

    with Path(output_dir, "target_gaits.json").open("w") as fp:
        json.dump(target_gaits, fp)


def smoothing(data_arrays, filter):
    for i in range(len(data_arrays)):
        for j in range(data_arrays[0].shape[1]):
            data_arrays[i][:, j] = np.convolve(filter, data_arrays[i][:, j], 'same')

    return data_arrays


def rolling_avg_smoothing(data_arrays, window_size):
    window = np.full(window_size, 1.0 / window_size)
    data_arrays = smoothing(data_arrays, window)

    return data_arrays


def gaussian_smoothing(data_arrays, sigma, size):
    x = np.linspace(-int(size / 2), int(size / 2), size)
    filter = (1 / np.sqrt(2 * np.pi * sigma ** 2)) * np.exp(-x ** 2 / sigma ** 2)

    data_arrays = smoothing(data_arrays, filter)

    return data_arrays


def load_endcap_data(npy_dir):
    npy_dir = Path(npy_dir)

    data = []
    np_array_paths = sorted([p for p in npy_dir.iterdir() if p.name[0].isdigit()],
                            key=lambda p: int(p.name[0]))
    for p in np_array_paths:
        endcap_data = np.load(p.as_posix())
        data.append(endcap_data)

    return data


def combine_endcaps_and_json_data(poses, end_caps_data, json_data):
    init_time = json_data[0]['header']['secs']

    data_jsons = []
    target_gaits = []
    for i in range(poses.shape[0]):
        gaits = [
            json_data[i]['motors']["0"]['target'],
            json_data[i]['motors']["1"]['target'],
            json_data[i]['motors']["2"]['target'],
            json_data[i]['motors']["3"]['target'],
            json_data[i]['motors']["4"]['target'],
            json_data[i]['motors']["5"]['target']
        ]
        controls = [
            json_data[i]['motors']["0"]['speed'] / json_data[i]['info']['max_speed'],
            json_data[i]['motors']["1"]['speed'] / json_data[i]['info']['max_speed'],
            json_data[i]['motors']["2"]['speed'] / json_data[i]['info']['max_speed'],
            json_data[i]['motors']["3"]['speed'] / json_data[i]['info']['max_speed'],
            json_data[i]['motors']["4"]['speed'] / json_data[i]['info']['max_speed'],
            json_data[i]['motors']["5"]['speed'] / json_data[i]['info']['max_speed']
        ]
        data = {
            "time": round(json_data[i]['header']['secs'] - init_time, 3),
            'pos': poses[i].tolist(),
            'rod_01_end_pt1': end_caps_data[0][i].tolist(),
            'rod_01_end_pt2': end_caps_data[1][i].tolist(),
            'rod_23_end_pt1': end_caps_data[2][i].tolist(),
            'rod_23_end_pt2': end_caps_data[3][i].tolist(),
            'rod_45_end_pt1': end_caps_data[4][i].tolist(),
            'rod_45_end_pt2': end_caps_data[5][i].tolist(),
            'end_pts': [e[i].tolist() for e in end_caps_data],
            'target_gaits': gaits,
            'controls': controls
        }
        data_jsons.append(data)

        if len(target_gaits) == 0 or gaits != target_gaits[-1]['target_gait']:
            target_gaits.append({
                'idx': i,
                'info': json_data[i]['info'],
                'target_gait': gaits
            })

    data_jsons = sorted(data_jsons, key=lambda d: d['time'])

    return data_jsons, target_gaits


def shift_end_caps(end_caps_data, end_cap_radius=0.0175, scale=10.0):
    end_cap_radius = scale * end_cap_radius

    e0 = end_caps_data[0] * scale
    e1 = end_caps_data[1] * scale
    e2 = end_caps_data[2] * scale
    e3 = end_caps_data[3] * scale
    e4 = end_caps_data[4] * scale
    e5 = end_caps_data[5] * scale

    prin01 = (e1 - e0) / np.linalg.norm(e1 - e0, axis=1)[:, None]
    prin23 = (e3 - e2) / np.linalg.norm(e3 - e2, axis=1)[:, None]
    prin45 = (e5 - e4) / np.linalg.norm(e5 - e4, axis=1)[:, None]

    e0 += (end_cap_radius + 0) * prin01
    e1 -= (end_cap_radius + 0) * prin01
    e2 += (end_cap_radius + 0) * prin23
    e3 -= (end_cap_radius + 0) * prin23
    e4 += (end_cap_radius + 0) * prin45
    e5 -= (end_cap_radius + 0) * prin45

    com = (e0[0] + e1[0] + e2[0] + e3[0] + e4[0] + e5[0]) / 6.0
    min_z = np.min([e0[0, 2], e1[0, 2], e2[0, 2], e3[0, 2], e4[0, 2], e5[0, 2]])
    shift = np.array([com[0], com[1], -end_cap_radius + min_z])

    e0 -= shift
    e1 -= shift
    e2 -= shift
    e3 -= shift
    e4 -= shift
    e5 -= shift

    return [e0, e1, e2, e3, e4, e5]


def load_raw_json_data(raw_data_dir):
    raw_data_dir = Path(raw_data_dir)

    data = []
    data_paths = sorted(raw_data_dir.glob("*.json"),
                        key=lambda p: int(p.name.split(".")[0]))
    for p in data_paths:
        with p.open('r') as j:
            data_json = json.load(j)
        data.append(data_json)

    return data


def get_camera_ext_mat(config_path):
    with Path(config_path).open("r") as j:
        config = json.load(j)

    ext_mat = np.array(config['cam_extr'])

    return ext_mat


def cam_frame_to_world_frame(endcaps_data, ext_mat):
    ext_mat_inv = ext_mat

    endcaps_data_world = []
    for endcap in endcaps_data:
        inhomo_endcap = np.hstack([endcap, np.ones((endcap.shape[0], 1))]).T
        endcap_world = np.matmul(ext_mat_inv, inhomo_endcap).T
        endcaps_data_world.append(endcap_world[:, :3])

    return endcaps_data_world


def linear_interp_seq(endcaps_data, raw_json_data, dt, rod_len=2.95):
    raw_json_data_aug = [(i, data) for i, data in enumerate(raw_json_data)]
    raw_json_data_aug = sorted(raw_json_data_aug, key=lambda d: d[1]['header']['secs'])

    raw_json_data = [r[1] for r in raw_json_data_aug]

    endcaps_data = [endcaps_data[r[0]] for r in raw_json_data_aug]

    init_time = raw_json_data_aug[0][1]['header']['secs']
    interp_data = [[] for _ in range(len(endcaps_data[0]))]

    for j in range(len(raw_json_data) - 1):
        t0 = raw_json_data[j]['header']['secs'] - init_time
        t1 = raw_json_data[j + 1]['header']['secs'] - init_time

        start = t0 if t0 == (t0 // dt) * dt else (t0 // dt + 1) * dt
        if t0 > start or start > t1:
            continue

        num_steps = int((t1 - start) // dt) + 1
        times = [start + i * dt for i in range(num_steps)]

        endcaps_interp = []
        for i in range(len(endcaps_data[0])):
            pt1 = endcaps_data[j][i]
            pt2 = endcaps_data[j + 1][i]
            pts = [linear_two_pts(t0, t1, pt1, pt2, t) for t in times]
            endcaps_interp.append(pts)

        for i in range(len(interp_data)):
            interp_data[i].extend(endcaps_interp[i])

    interp_data = [np.vstack(x) for x in interp_data]
    # interp_data = rolling_avg_smoothing(interp_data, 30)
    for j in range(len(interp_data) // 2):
        pt1 = interp_data[2 * j]
        pt2 = interp_data[2 * j + 1]

        com = (pt1 + pt2) / 2.
        prin = (pt2 - pt1) / np.linalg.norm(pt2 - pt1)

        pt1 = com - rod_len * prin / 2.
        pt2 = com + rod_len * prin / 2.

        interp_data[2 * j] = pt1
        interp_data[2 * j + 1] = pt2

    return interp_data


def linear_two_pts_seq(t1, t2, pt1, pt2):
    delta_t = t2 - t1
    delta_pt = pt2 - pt1

    pts = [(delta_pt / delta_t) * t_i + pt2 - (delta_pt / delta_t) * t2
           for t_i in range(t1 + 1, t2)] + [pt2]

    return pts


def linear_two_pts(t1, t2, pt1, pt2, t):
    dt = t2 - t1
    w1 = np.abs(t2 - t) / dt
    w2 = 1 - w1

    pt = pt1 * w1 + pt2 * w2

    return pt


def compute_poses(data_arrays):
    """
    Assume data arrays are arranged such that i (even) and i+1 (odd) is end pts of the same rod e.g (0, 1) or (2, 3)
    :param data_arrays:
    :param scaling_factor:
    :param min_z:
    :return:
    """
    poses = []

    scaling_factor = 1.0
    offset = np.array([0.0, 0.0, 0.0])

    for i in range(0, len(data_arrays), 2):
        data1 = scaling_factor * data_arrays[i] - offset
        data2 = scaling_factor * data_arrays[i + 1] - offset

        com = (data1 + data2) / 2.0

        principal_axis = data2 - data1
        unit_prin_axis = principal_axis / np.linalg.norm(principal_axis, axis=1).reshape(-1, 1)
        quat = np.hstack([1 + unit_prin_axis[:, 2:3],
                          -unit_prin_axis[:, 1:2],
                          unit_prin_axis[:, 0:1],
                          np.zeros((unit_prin_axis.shape[0], 1))])
        quat /= np.linalg.norm(quat, axis=1).reshape(-1, 1)

        pose = np.hstack([com, quat])
        poses.append(pose)

    poses_array = np.hstack(poses)

    return poses_array


def lerp_fn(pt1, pt2, t1, t2):
    slopes = [(pt2[i] - pt1[i]) / (t2 - t1) for i in range(len(pt1))]
    bias = [pt1[i] - slopes[i] * t1 for i in range(len(pt1))]

    return lambda t: [slopes[i] * t + bias[i] for i in range(len(pt1))]


# def slerp_fn(q1, q2, t1, t2):
#     q3 = torch_quaternion.quat_prod(q2, q1)
#     return torch_quaternion.quat_pow(q3, )


if __name__ == '__main__':
    # for i in range(2, 5):
    dataset_name = f"sysIDnew_14"
    base_path = Path("/Users/nelsonchen/Documents/Rutgers-CS-PhD/Research/tensegrity/data_sets/"
                     f"tensegrity_real_datasets/RSS_demo_new_platform/{dataset_name}")
    data_path = Path(base_path, 'poses-proposed')  # data folder called "poses"
    config_path = Path(base_path, 'config.json')  # config file location
    raw_data_path = Path(base_path, 'data')  # data folder called "data
    output_path = Path(base_path / f"./")  # output directory
    dt = 0.01

    # Generates one json file for the raw data and one json file with linearly
    # interpolated data based on dt specified
    process_real_data(data_path, config_path, raw_data_path, output_path, dt)
