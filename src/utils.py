import numpy as np
from astar import rel_mov, angle_norm

def rotate_quaternion_around_z(quaternion, angle):
    """
    Rotates a quaternion around the z-axis by a given angle.

    Parameters:
        quaternion (tuple): The input quaternion (w, x, y, z).
        angle (float): The rotation angle in radians.

    Returns:
        tuple: The rotated quaternion (w, x, y, z).
    """
    # Decompose the input quaternion
    w, x, y, z = quaternion
    
    # Create the rotation quaternion for the z-axis
    half_angle = angle / 2
    rot_w = np.cos(half_angle)
    rot_x = 0
    rot_y = 0
    rot_z = np.sin(half_angle)
    rotation_quaternion = (rot_w, rot_x, rot_y, rot_z)
    
    # Multiply the two quaternions: rotation_quaternion * quaternion
    rw, rx, ry, rz = rotation_quaternion
    
    # Quaternion multiplication formula
    new_w = rw * w - rx * x - ry * y - rz * z
    new_x = rw * x + rx * w + ry * z - rz * y
    new_y = rw * y - rx * z + ry * w + rz * x
    new_z = rw * z + rx * y - ry * x + rz * w
    
    # Return the rotated quaternion
    return [new_w, new_x, new_y, new_z]

def quaternion_rotation_z(quaternion):
    """
    Extracts the rotation around the z-axis from a quaternion.

    Parameters:
        quaternion (tuple): The input quaternion (w, x, y, z).

    Returns:
        float: The angle of rotation around the z-axis in radians.
    """
    w, x, y, z = quaternion

    # Calculate the rotation angle around the z-axis
    # Assumes quaternion is normalized
    angle_z = 2 * np.arctan2(z, w)

    # Normalize the angle to be within the range [-pi, pi]
    if angle_z > np.pi:
        angle_z -= 2 * np.pi
    elif angle_z < -np.pi:
        angle_z += 2 * np.pi

    return angle_z


def gaits_to_path(start, gait_path, gaits):
    path = []
    curr = start
    for g in gait_path:
        gait = gaits[g]
        dx, dy = rel_mov(gait[0], gait[1], curr[2])
        curr = (curr[0]+dx, curr[1]+dy, angle_norm(curr[2]+gait[3]))
        path.append(curr)

    return path