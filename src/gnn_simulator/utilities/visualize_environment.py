"""
Visualize environment objects (StaticPrism, StaticRectPlane) from a simulator config JSON.

Usage:
    python -m gnn_simulator.utilities.visualize_environment <config.json>

Example:
    python -m gnn_simulator.utilities.visualize_environment \
        gnn_simulator/simulators/configs/new_3_bar_15_cables_5d_gnn_sim_3d_obs_course_config.json
"""

import argparse
import json
import sys

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# 8 corners of a unit cube centered at origin
_UNIT_CUBE = np.array([
    [-1, -1, -1],
    [+1, -1, -1],
    [+1, +1, -1],
    [-1, +1, -1],
    [-1, -1, +1],
    [+1, -1, +1],
    [+1, +1, +1],
    [-1, +1, +1],
], dtype=float)

# 6 faces as vertex index quads
_CUBE_FACES = [
    [0, 1, 2, 3],  # -z
    [4, 5, 6, 7],  # +z
    [0, 1, 5, 4],  # -y
    [2, 3, 7, 6],  # +y
    [0, 3, 7, 4],  # -x
    [1, 2, 6, 5],  # +x
]

# 4 corners of a unit square in the x-y plane
_UNIT_RECT = np.array([
    [-1, -1, 0],
    [+1, -1, 0],
    [+1, +1, 0],
    [-1, +1, 0],
], dtype=float)


def _prism_vertices(pos, rot_mat, half_lens):
    """Return (8, 3) world-frame vertices for a StaticPrism."""
    pos = np.asarray(pos)
    rot_mat = np.asarray(rot_mat)
    half_lens = np.asarray(half_lens)
    verts = _UNIT_CUBE * half_lens  # scale
    verts = verts @ rot_mat.T        # rotate
    verts = verts + pos              # translate
    return verts


def _rect_plane_vertices(pos, rot_mat, half_lens):
    """Return (4, 3) world-frame vertices for a StaticRectPlane."""
    pos = np.asarray(pos)
    rot_mat = np.asarray(rot_mat)
    scale = np.array([half_lens[0], half_lens[1], 0.0])
    verts = _UNIT_RECT * scale
    verts = verts @ rot_mat.T
    verts = verts + pos
    return verts


def _add_prism(ax, name, pos, rot_mat, half_lens, color, alpha=0.25):
    verts = _prism_vertices(pos, rot_mat, half_lens)
    faces = [[verts[i] for i in face] for face in _CUBE_FACES]
    poly = Poly3DCollection(faces, alpha=alpha, facecolor=color, edgecolor='k', linewidth=0.5)
    ax.add_collection3d(poly)
    ax.text(*np.asarray(pos), name, fontsize=7, ha='center', va='center')


def _add_rect_plane(ax, name, pos, rot_mat, half_lens, color, alpha=0.35):
    verts = _rect_plane_vertices(pos, rot_mat, half_lens)
    poly = Poly3DCollection([verts], alpha=alpha, facecolor=color, edgecolor='k', linewidth=0.8)
    ax.add_collection3d(poly)
    ax.text(*np.asarray(pos), name, fontsize=7, ha='center', va='center')


def visualize_environment(config_path: str):
    with open(config_path, 'r') as f:
        cfg = json.load(f)

    env = cfg.get("environment")
    if env is None:
        print(f"No 'environment' key found in {config_path}")
        sys.exit(1)

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Color palette
    prism_colors = plt.cm.Set3(np.linspace(0, 1, 12))
    plane_colors = plt.cm.Pastel1(np.linspace(0, 1, 9))
    prism_idx, plane_idx = 0, 0

    all_pts = []

    for name, obj in env.items():
        obj_type = obj["type"]
        pos = obj["pos"]
        rot_mat = obj["rot_mat"]
        half_lens = obj["half_lens"]

        if obj_type == "StaticPrism":
            color = prism_colors[prism_idx % len(prism_colors)]
            prism_idx += 1
            _add_prism(ax, name, pos, rot_mat, half_lens, color)
            verts = _prism_vertices(pos, rot_mat, half_lens)
            all_pts.append(verts)

        elif obj_type == "StaticRectPlane":
            color = plane_colors[plane_idx % len(plane_colors)]
            plane_idx += 1
            _add_rect_plane(ax, name, pos, rot_mat, half_lens, color)
            verts = _rect_plane_vertices(pos, rot_mat, half_lens)
            all_pts.append(verts)

        else:
            print(f"Warning: unknown type '{obj_type}' for '{name}', skipping")

    # Auto-fit axes
    if all_pts:
        all_pts = np.concatenate(all_pts, axis=0)
        margin = 1.0
        mins = all_pts.min(axis=0) - margin
        maxs = all_pts.max(axis=0) + margin

        # Equal aspect ratio
        ranges = maxs - mins
        max_range = ranges.max()
        centers = (mins + maxs) / 2
        ax.set_xlim(centers[0] - max_range / 2, centers[0] + max_range / 2)
        ax.set_ylim(centers[1] - max_range / 2, centers[1] + max_range / 2)
        ax.set_zlim(centers[2] - max_range / 2, centers[2] + max_range / 2)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(config_path.split('/')[-1])
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize environment from simulator config")
    parser.add_argument("config", help="Path to simulator config JSON file")
    args = parser.parse_args()
    visualize_environment(args.config)
