"""
Contact detection utilities for computing signed distances between primitive shapes.
All functions support batched PyTorch tensors and return:
- point_on_obj1: closest point on object 1's surface (batch_size, 3, 1)
- point_on_obj2: closest point on object 2's surface (batch_size, 3, 1)
- signed_distance: signed distance between the two surfaces (batch_size, 1)
  (negative = penetration, positive = separation)
- normal_on_obj1: outward normal vector at point_on_obj1 (batch_size, 3, 1)
- normal_on_obj2: outward normal vector at point_on_obj2 (batch_size, 3, 1)
"""

import torch
from typing import Tuple, Callable
from gnn_simulator.state_objects.primitive_shapes import SphereState, Cylinder, StaticPrism, StaticRectPlane
from gnn_simulator.state_objects.rigid_object import RigidBody


def sphere_sphere_signed_distance(
        sphere1: SphereState,
        sphere2: SphereState
) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """
    Compute signed distance between two spheres.

    Args:
        sphere1: First sphere object
        sphere2: Second sphere object

    Returns:
        point_on_sphere1: Closest point on sphere1 surface (batch_size, 3, 1)
        point_on_sphere2: Closest point on sphere2 surface (batch_size, 3, 1)
        signed_distance: Signed distance (batch_size, 1)
        normal_on_sphere1: Outward normal at point_on_sphere1 (batch_size, 3, 1)
        normal_on_sphere2: Outward normal at point_on_sphere2 (batch_size, 3, 1)
    """
    # Get centers and radii - keep shape (batch_size, 3, 1)
    center1 = sphere1.pos  # (batch_size, 3, 1)
    center2 = sphere2.pos  # (batch_size, 3, 1)
    batch_size = center1.shape[0]
    # Handle radius - may be (1, 1, 1) or (batch_size, 1, 1)
    radius1_raw = sphere1.radius
    radius2_raw = sphere2.radius
    # Squeeze to get (batch_size,) or scalar, expanding if needed
    if radius1_raw.shape[0] == 1 and batch_size > 1:
        radius1 = radius1_raw.expand(batch_size, -1, -1).squeeze(-1).squeeze(-1)  # (batch_size,)
    else:
        radius1 = radius1_raw.squeeze(-1).squeeze(-1)  # (batch_size,) or scalar
    if radius2_raw.shape[0] == 1 and batch_size > 1:
        radius2 = radius2_raw.expand(batch_size, -1, -1).squeeze(-1).squeeze(-1)  # (batch_size,)
    else:
        radius2 = radius2_raw.squeeze(-1).squeeze(-1)  # (batch_size,) or scalar

    # Ensure radius1 and radius2 are 1D tensors for proper broadcasting
    if len(radius1.shape) == 0:
        radius1 = radius1.unsqueeze(0)
    if len(radius2.shape) == 0:
        radius2 = radius2.unsqueeze(0)
    if radius1.shape[0] == 1 and batch_size > 1:
        radius1 = radius1.expand(batch_size)
    if radius2.shape[0] == 1 and batch_size > 1:
        radius2 = radius2.expand(batch_size)

    # Vector from center1 to center2
    vec_12 = center2 - center1  # (batch_size, 3, 1)
    dist_centers = torch.linalg.norm(vec_12, dim=1, keepdim=True)  # (batch_size, 1, 1)
    dist_centers = dist_centers.squeeze(-1)  # (batch_size, 1)

    # Avoid division by zero
    eps = 1e-8
    unit_vec = vec_12 / (dist_centers.unsqueeze(-1) + eps)  # (batch_size, 3, 1)

    # Closest points on surfaces
    radius1_expanded = radius1.unsqueeze(-1).unsqueeze(-1)  # (batch_size, 1, 1)
    radius2_expanded = radius2.unsqueeze(-1).unsqueeze(-1)  # (batch_size, 1, 1)
    point_on_sphere1 = center1 + unit_vec * radius1_expanded  # (batch_size, 3, 1)
    point_on_sphere2 = center2 - unit_vec * radius2_expanded  # (batch_size, 3, 1)

    # Signed distance: negative if penetrating, positive if separated
    radius_sum = radius1.unsqueeze(-1) + radius2.unsqueeze(-1)  # Should be (batch_size, 1)
    signed_distance = dist_centers - radius_sum  # (batch_size, 1)
    # Ensure shape is (batch_size, 1) - squeeze any extra trailing dimensions
    while len(signed_distance.shape) > 2:
        signed_distance = signed_distance.squeeze(-1)
    if len(signed_distance.shape) == 1:
        signed_distance = signed_distance.unsqueeze(-1)  # Ensure (batch_size, 1)

    # Normals: for spheres, the normal at the closest point is the unit vector
    # pointing from center to the point (outward)
    normal_on_sphere1 = unit_vec  # Points from center1 toward center2 (outward from sphere1)
    normal_on_sphere2 = -unit_vec  # Points from center2 toward center1 (outward from sphere2)

    return point_on_sphere1, point_on_sphere2, signed_distance, normal_on_sphere1, normal_on_sphere2


def sphere_cylinder_signed_distance(
        sphere: SphereState,
        cylinder: Cylinder
) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """
    Compute signed distance between a sphere and a cylinder (excluding endcaps).

    Args:
        sphere: Sphere object
        cylinder: Cylinder object

    Returns:
        point_on_sphere: Closest point on sphere surface (batch_size, 3, 1)
        point_on_cylinder: Closest point on cylinder surface (batch_size, 3, 1)
        signed_distance: Signed distance (batch_size, 1)
        normal_on_sphere: Outward normal at point_on_sphere (batch_size, 3, 1)
        normal_on_cylinder: Outward normal at point_on_cylinder (batch_size, 3, 1)
    """
    # Get sphere center and radius - keep shape (batch_size, 3, 1)
    sphere_center = sphere.pos  # (batch_size, 3, 1)
    # Get 1D radius for calculations
    sphere_radius_raw = sphere.radius.squeeze(-1).squeeze(-1)  # (batch_size,) or scalar
    if len(sphere_radius_raw.shape) == 0:
        sphere_radius_raw = sphere_radius_raw.unsqueeze(0)
    sphere_radius = sphere_radius_raw.unsqueeze(-1).unsqueeze(-1)  # (batch_size, 1, 1)

    # Get cylinder end points and radius - keep shape (batch_size, 3, 1)
    end_pts = cylinder._compute_end_pts()  # [end_pt1, end_pt2], each (batch_size, 3, 1)
    end_pt1 = end_pts[0]  # (batch_size, 3, 1)
    end_pt2 = end_pts[1]  # (batch_size, 3, 1)
    cylinder_radius_raw = cylinder.radius.squeeze(-1).squeeze(-1)  # (batch_size,) or scalar
    if len(cylinder_radius_raw.shape) == 0:
        cylinder_radius_raw = cylinder_radius_raw.unsqueeze(0)
    cylinder_radius = cylinder_radius_raw.unsqueeze(-1).unsqueeze(-1)  # (batch_size, 1, 1)

    # Cylinder axis direction
    axis = end_pt2 - end_pt1  # (batch_size, 3, 1)
    axis_length_3d = torch.linalg.norm(axis, dim=1, keepdim=True)  # (batch_size, 1, 1)
    axis_length = axis_length_3d.squeeze(-1)  # (batch_size, 1) for clamp
    axis_unit = axis / axis_length_3d  # (batch_size, 3, 1)

    # Vector from end_pt1 to sphere center
    vec_to_sphere = sphere_center - end_pt1  # (batch_size, 3, 1)

    # Project sphere center onto cylinder axis
    proj_length = torch.sum(vec_to_sphere * axis_unit, dim=1, keepdim=True)  # (batch_size, 1, 1)
    proj_length = proj_length.squeeze(-1)  # (batch_size, 1)

    # Clamp to cylinder length (excluding endcaps - so we clamp to [0, axis_length])
    # Use tensor for min to match max type
    min_val = torch.zeros_like(axis_length)  # (batch_size, 1)
    proj_length_clamped = torch.clamp(proj_length, min=min_val, max=axis_length)

    # Closest point on cylinder axis
    closest_on_axis = end_pt1 + proj_length_clamped.unsqueeze(-1) * axis_unit  # (batch_size, 3, 1)

    # Vector from closest point on axis to sphere center
    vec_to_sphere_from_axis = sphere_center - closest_on_axis  # (batch_size, 3, 1)
    dist_to_axis = torch.linalg.norm(vec_to_sphere_from_axis, dim=1, keepdim=True)  # (batch_size, 1, 1)
    dist_to_axis = dist_to_axis.squeeze(-1)  # (batch_size, 1)

    # Closest point on cylinder surface
    eps = 1e-8
    unit_vec_to_sphere = vec_to_sphere_from_axis / (dist_to_axis.unsqueeze(-1) + eps)  # Add eps to avoid div by zero
    point_on_cylinder = closest_on_axis + unit_vec_to_sphere * cylinder_radius

    # Closest point on sphere surface (toward cylinder)
    vec_sphere_to_cylinder = point_on_cylinder - sphere_center  # (batch_size, 3, 1)
    dist_sphere_to_cylinder = torch.linalg.norm(vec_sphere_to_cylinder, dim=1, keepdim=True)  # (batch_size, 1, 1)
    dist_sphere_to_cylinder = dist_sphere_to_cylinder.squeeze(-1)  # (batch_size, 1)
    unit_vec_sphere = vec_sphere_to_cylinder / (
                dist_sphere_to_cylinder.unsqueeze(-1) + eps)  # Add eps to avoid div by zero
    point_on_sphere = sphere_center + unit_vec_sphere * sphere_radius

    # Signed distance: dist_between_surfaces is already the distance between surface points
    # which equals (dist_to_axis - cylinder_radius - sphere_radius) when separated
    # We need to compute: (distance between centers on surface normal) - sphere_radius - cylinder_radius
    # Which is: dist_to_axis - cylinder_radius - sphere_radius
    signed_distance = dist_to_axis - (sphere_radius_raw.unsqueeze(-1) + cylinder_radius_raw.unsqueeze(-1))
    # Ensure shape is (batch_size, 1)
    while len(signed_distance.shape) > 2:
        signed_distance = signed_distance.squeeze(-1)
    if len(signed_distance.shape) == 1:
        signed_distance = signed_distance.unsqueeze(-1)

    # Normals:
    # Sphere normal: outward from sphere center
    normal_on_sphere = unit_vec_sphere  # Points from sphere center toward cylinder (outward)
    # Cylinder normal: perpendicular to axis, pointing outward from cylinder
    normal_on_cylinder = unit_vec_to_sphere  # Points from cylinder axis toward sphere (outward)

    return point_on_sphere, point_on_cylinder, signed_distance, normal_on_sphere, normal_on_cylinder


def cylinder_cylinder_signed_distance(
        cylinder1: Cylinder,
        cylinder2: Cylinder
) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """
    Compute signed distance between two cylinders (excluding endcaps).

    Args:
        cylinder1: First cylinder object
        cylinder2: Second cylinder object

    Returns:
        point_on_cylinder1: Closest point on cylinder1 surface (batch_size, 3, 1)
        point_on_cylinder2: Closest point on cylinder2 surface (batch_size, 3, 1)
        signed_distance: Signed distance (batch_size, 1)
        normal_on_cylinder1: Outward normal at point_on_cylinder1 (batch_size, 3, 1)
        normal_on_cylinder2: Outward normal at point_on_cylinder2 (batch_size, 3, 1)
    """
    # Get cylinder end points and radii - keep shape (batch_size, 3, 1)
    end_pts1 = cylinder1._compute_end_pts()
    end_pt1_1 = end_pts1[0]  # (batch_size, 3, 1)
    end_pt1_2 = end_pts1[1]  # (batch_size, 3, 1)
    radius1_raw = cylinder1.radius.squeeze(-1).squeeze(-1)  # (batch_size,) or scalar

    end_pts2 = cylinder2._compute_end_pts()
    end_pt2_1 = end_pts2[0]  # (batch_size, 3, 1)
    end_pt2_2 = end_pts2[1]  # (batch_size, 3, 1)
    radius2_raw = cylinder2.radius.squeeze(-1).squeeze(-1)  # (batch_size,) or scalar

    # Ensure radii are at least 1D for broadcasting
    batch_size = end_pt1_1.shape[0]
    if len(radius1_raw.shape) == 0:
        radius1_raw = radius1_raw.unsqueeze(0)
    if len(radius2_raw.shape) == 0:
        radius2_raw = radius2_raw.unsqueeze(0)
    if radius1_raw.shape[0] == 1 and batch_size > 1:
        radius1_raw = radius1_raw.expand(batch_size)
    if radius2_raw.shape[0] == 1 and batch_size > 1:
        radius2_raw = radius2_raw.expand(batch_size)

    # For point calculations, need (batch_size, 1, 1)
    radius1 = radius1_raw.unsqueeze(-1).unsqueeze(-1)  # (batch_size, 1, 1)
    radius2 = radius2_raw.unsqueeze(-1).unsqueeze(-1)  # (batch_size, 1, 1)

    # Cylinder axes
    axis1 = end_pt1_2 - end_pt1_1  # (batch_size, 3, 1)
    axis2 = end_pt2_2 - end_pt2_1  # (batch_size, 3, 1)

    axis1_length = torch.linalg.norm(axis1, dim=1, keepdim=True)  # (batch_size, 1, 1)
    axis1_length = axis1_length.squeeze(-1)  # (batch_size, 1)
    axis2_length = torch.linalg.norm(axis2, dim=1, keepdim=True)  # (batch_size, 1, 1)
    axis2_length = axis2_length.squeeze(-1)  # (batch_size, 1)

    eps = 1e-8
    axis1_unit = axis1 / (axis1_length.unsqueeze(-1) + eps)  # (batch_size, 3, 1)
    axis2_unit = axis2 / (axis2_length.unsqueeze(-1) + eps)  # (batch_size, 3, 1)

    # Find closest points between the two line segments
    # Using parametric representation with unit vectors:
    # Line 1: p1(s) = end_pt1_1 + s * axis1_unit, s in [0, axis1_length]
    # Line 2: p2(t) = end_pt2_1 + t * axis2_unit, t in [0, axis2_length]

    w = end_pt1_1 - end_pt2_1  # (batch_size, 3, 1)
    axis1_unit_2d = axis1_unit.squeeze(-1)  # (batch_size, 3) for dot products
    axis2_unit_2d = axis2_unit.squeeze(-1)  # (batch_size, 3) for dot products
    w_2d = w.squeeze(-1)  # (batch_size, 3) for dot products

    a = torch.sum(axis1_unit_2d * axis1_unit_2d, dim=1, keepdim=True)  # (batch_size, 1) = 1
    b = torch.sum(axis1_unit_2d * axis2_unit_2d, dim=1, keepdim=True)  # (batch_size, 1)
    c = torch.sum(axis2_unit_2d * axis2_unit_2d, dim=1, keepdim=True)  # (batch_size, 1) = 1
    d = torch.sum(axis1_unit_2d * w_2d, dim=1, keepdim=True)  # (batch_size, 1)
    e = torch.sum(axis2_unit_2d * w_2d, dim=1, keepdim=True)  # (batch_size, 1)

    denom = a * c - b * b  # (batch_size, 1)

    # Handle parallel and non-parallel lines
    parallel_threshold = 1e-6
    is_parallel = torch.abs(denom) < parallel_threshold

    # For non-parallel lines: solve for optimal s and t
    s_unclamped = (b * e - c * d) / (denom + eps)  # (batch_size, 1)
    t_unclamped = (a * e - b * d) / (denom + eps)  # (batch_size, 1)

    # Clamp parameters and refine
    min_val_s = torch.zeros_like(axis1_length)  # (batch_size, 1)
    min_val_t = torch.zeros_like(axis2_length)  # (batch_size, 1)

    # Initial clamping
    s_clamped = torch.clamp(s_unclamped, min=min_val_s, max=axis1_length)
    t_clamped = torch.clamp(t_unclamped, min=min_val_t, max=axis2_length)

    # Refine: if s was clamped, recompute optimal t for that s, and vice versa
    s_was_clamped = (s_unclamped != s_clamped)
    t_was_clamped = (t_unclamped != t_clamped)

    # Recompute t given clamped s: t = (b*s + e)
    t_refined = b * s_clamped + e
    t_refined = torch.clamp(t_refined, min=min_val_t, max=axis2_length)

    # Recompute s given clamped t: s = (b*t - d)
    s_refined = b * t_clamped - d
    s_refined = torch.clamp(s_refined, min=min_val_s, max=axis1_length)

    # Use refined values where appropriate
    s_final = torch.where(s_was_clamped, s_clamped, s_refined)
    t_final = torch.where(t_was_clamped, t_refined, t_clamped)

    # For parallel lines: check all four endpoint combinations
    # NOTE: Always compute the parallel path (no data-dependent `if`) so that
    # torch.compile / torch._dynamo can trace through without graph breaks.
    # The results are masked via torch.where(is_parallel, ...) below.

    # Compute distances for all 4 endpoint pairs
    # (end_pt1_1, end_pt2_1), (end_pt1_1, end_pt2_2), (end_pt1_2, end_pt2_1), (end_pt1_2, end_pt2_2)

    # Project all endpoints of cylinder2 onto cylinder1 axis
    vec_to_end2_1 = end_pt2_1 - end_pt1_1  # (batch_size, 3, 1)
    vec_to_end2_2 = end_pt2_2 - end_pt1_1  # (batch_size, 3, 1)

    proj_2_1 = torch.sum(axis1_unit_2d * vec_to_end2_1.squeeze(-1), dim=1, keepdim=True)  # (batch_size, 1)
    proj_2_2 = torch.sum(axis1_unit_2d * vec_to_end2_2.squeeze(-1), dim=1, keepdim=True)  # (batch_size, 1)

    # Clamp projections to cylinder1 length
    s_par_1 = torch.clamp(proj_2_1, min=min_val_s, max=axis1_length)
    s_par_2 = torch.clamp(proj_2_2, min=min_val_s, max=axis1_length)

    # Project endpoints of cylinder1 onto cylinder2 axis
    vec_to_end1_1 = end_pt1_1 - end_pt2_1  # (batch_size, 3, 1)
    vec_to_end1_2 = end_pt1_2 - end_pt2_1  # (batch_size, 3, 1)

    proj_1_1 = torch.sum(axis2_unit_2d * vec_to_end1_1.squeeze(-1), dim=1, keepdim=True)  # (batch_size, 1)
    proj_1_2 = torch.sum(axis2_unit_2d * vec_to_end1_2.squeeze(-1), dim=1, keepdim=True)  # (batch_size, 1)

    # Clamp projections to cylinder2 length
    t_par_1 = torch.clamp(proj_1_1, min=min_val_t, max=axis2_length)
    t_par_2 = torch.clamp(proj_1_2, min=min_val_t, max=axis2_length)

    # Compute closest points for both projections and choose minimum
    pt1_1 = end_pt1_1 + s_par_1.unsqueeze(-1) * axis1_unit
    pt1_2 = end_pt1_1 + s_par_2.unsqueeze(-1) * axis1_unit
    pt2_1 = end_pt2_1 + t_par_1.unsqueeze(-1) * axis2_unit
    pt2_2 = end_pt2_1 + t_par_2.unsqueeze(-1) * axis2_unit

    dist_1 = torch.linalg.norm(pt1_1 - end_pt2_1, dim=1, keepdim=True).squeeze(-1)
    dist_2 = torch.linalg.norm(pt1_2 - end_pt2_2, dim=1, keepdim=True).squeeze(-1)
    dist_3 = torch.linalg.norm(end_pt1_1 - pt2_1, dim=1, keepdim=True).squeeze(-1)
    dist_4 = torch.linalg.norm(end_pt1_2 - pt2_2, dim=1, keepdim=True).squeeze(-1)

    # Choose the configuration with minimum distance
    use_1 = (dist_1 <= dist_2) & (dist_1 <= dist_3) & (dist_1 <= dist_4)
    use_2 = (dist_2 < dist_1) & (dist_2 <= dist_3) & (dist_2 <= dist_4)
    use_3 = (dist_3 < dist_1) & (dist_3 < dist_2) & (dist_3 <= dist_4)

    s_parallel = torch.where(use_1 | use_2, s_par_1, min_val_s)
    s_parallel = torch.where(~(use_1 | use_2) & ~use_3, axis1_length, s_parallel)
    s_parallel = torch.where(use_2, s_par_2, s_parallel)

    t_parallel = torch.where(use_3 | use_1, torch.zeros_like(t_par_1), t_par_1)
    t_parallel = torch.where(~(use_1 | use_3) & use_2, axis2_length, t_parallel)
    t_parallel = torch.where(~use_1 & ~use_2 & ~use_3, axis2_length, t_parallel)

    # Use parallel values where lines are parallel
    s_final = torch.where(is_parallel, s_parallel, s_final)
    t_final = torch.where(is_parallel, t_parallel, t_final)

    # Closest points on axes
    closest_on_axis1 = end_pt1_1 + s_final.unsqueeze(-1) * axis1_unit  # (batch_size, 3, 1)
    closest_on_axis2 = end_pt2_1 + t_final.unsqueeze(-1) * axis2_unit  # (batch_size, 3, 1)

    # Vector between closest points on axes
    vec_between_axes = closest_on_axis2 - closest_on_axis1  # (batch_size, 3, 1)
    dist_between_axes = torch.linalg.norm(vec_between_axes, dim=1, keepdim=True)  # (batch_size, 1, 1)
    dist_between_axes_2d = dist_between_axes.squeeze(-1)  # (batch_size, 1)

    # Find closest points on cylinder surfaces
    unit_vec = vec_between_axes / (dist_between_axes + eps)  # Add eps to avoid div by zero
    point_on_cylinder1 = closest_on_axis1 + unit_vec * radius1
    point_on_cylinder2 = closest_on_axis2 - unit_vec * radius2

    # Signed distance: distance between axes minus both radii
    # Positive if separated, negative if penetrating
    signed_distance = dist_between_axes_2d - (radius1_raw.unsqueeze(-1) + radius2_raw.unsqueeze(-1))

    # Ensure shape is (batch_size, 1)
    while len(signed_distance.shape) > 2:
        signed_distance = signed_distance.squeeze(-1)
    if len(signed_distance.shape) == 1:
        signed_distance = signed_distance.unsqueeze(-1)

    # Normals: perpendicular to cylinder axes, pointing outward
    normal_on_cylinder1 = unit_vec  # Points from axis1 toward axis2 (outward from cylinder1)
    normal_on_cylinder2 = -unit_vec  # Points from axis2 toward axis1 (outward from cylinder2)

    return point_on_cylinder1, point_on_cylinder2, signed_distance, normal_on_cylinder1, normal_on_cylinder2


def sphere_static_prism_signed_distance(
        sphere: SphereState,
        prism: StaticPrism
) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """
    Compute signed distance between a sphere and a static prism.

    Args:
        sphere: Sphere object
        prism: StaticPrism object

    Returns:
        point_on_sphere: Closest point on sphere surface (batch_size, 3, 1)
        point_on_prism: Closest point on prism surface (batch_size, 3, 1)
        signed_distance: Signed distance (batch_size, 1)
        normal_on_sphere: Outward normal at point_on_sphere (batch_size, 3, 1)
        normal_on_prism: Outward normal at point_on_prism (batch_size, 3, 1)
    """
    sphere_center = sphere.pos  # (batch_size, 3, 1)
    batch_size = sphere_center.shape[0]

    # Get sphere radius
    sphere_radius_raw = sphere.radius.squeeze(-1).squeeze(-1)  # (batch_size,) or scalar
    if len(sphere_radius_raw.shape) == 0:
        sphere_radius_raw = sphere_radius_raw.unsqueeze(0)
    if sphere_radius_raw.shape[0] == 1 and batch_size > 1:
        sphere_radius_raw = sphere_radius_raw.expand(batch_size)
    sphere_radius = sphere_radius_raw.unsqueeze(-1).unsqueeze(-1)  # (batch_size, 1, 1)

    # Transform sphere center to prism's body frame
    # prism.rot_mat is (batch_size, 3, 3), prism.pos is (batch_size, 3, 1)
    prism_pos = prism.pos  # (batch_size, 3, 1)
    rot_mat_inv = prism.rot_mat.transpose(1, 2)  # (batch_size, 3, 3)

    sphere_center_body = torch.einsum("bij,bjk->bik", rot_mat_inv, sphere_center - prism_pos)  # (batch_size, 3, 1)
    sphere_center_body_2d = sphere_center_body.squeeze(-1)  # (batch_size, 3) for operations

    # Get prism half lengths - handle both (1, 3, 1) and (batch_size, 3, 1) shapes
    half_lens = prism.half_lens.squeeze(-1)  # (1, 3) or (batch_size, 3)
    if half_lens.shape[0] == 1 and batch_size > 1:
        half_lens = half_lens.expand(batch_size, -1)  # (batch_size, 3)

    # Find closest point on prism in body frame
    # Clamp to prism bounds
    closest_body_2d = torch.clamp(sphere_center_body_2d,
                                  min=-half_lens,
                                  max=half_lens)  # (batch_size, 3)

    # Check if sphere center is inside or outside prism
    dist_to_surface_body = torch.abs(sphere_center_body_2d) - half_lens  # (batch_size, 3)
    max_dist = torch.max(dist_to_surface_body, dim=1, keepdim=True)[0]  # (batch_size, 1)

    # If inside, find the closest face (vectorized)
    is_inside = (max_dist < 0).squeeze(-1)  # (batch_size,)

    # NOTE: Always compute the inside path (no data-dependent `if`) so that
    # torch.compile / torch._dynamo can trace through without graph breaks.
    # The results are masked via torch.where(is_inside, ...) below.

    # Find the face with minimum distance
    min_dist_idx = torch.argmin(torch.abs(dist_to_surface_body), dim=1)  # (batch_size,)

    # Vectorized: project onto the closest face using one-hot encoding
    one_hot = torch.nn.functional.one_hot(min_dist_idx, num_classes=3).float()  # (batch_size, 3)

    # For points inside, set the coordinate on the closest face to the boundary
    closest_body_inside_2d = torch.where(
        one_hot.bool(),
        torch.sign(sphere_center_body_2d) * half_lens,
        sphere_center_body_2d
    )  # (batch_size, 3)

    # Use inside points only where is_inside is True
    closest_body_2d = torch.where(
        is_inside.unsqueeze(-1),
        closest_body_inside_2d,
        closest_body_2d
    )  # (batch_size, 3)

    # Transform back to world frame
    closest_body = closest_body_2d.unsqueeze(-1)  # (batch_size, 3, 1)
    point_on_prism = torch.einsum("bij,bjk->bik", prism.rot_mat, closest_body) + prism_pos  # (batch_size, 3, 1)

    # Vector from prism point to sphere center
    vec_prism_to_sphere = sphere_center - point_on_prism  # (batch_size, 3, 1)
    dist_center_to_prism = torch.linalg.norm(vec_prism_to_sphere, dim=1, keepdim=True)  # (batch_size, 1, 1)
    dist_center_to_prism = dist_center_to_prism.squeeze(-1)  # (batch_size, 1)

    # Unit vector from prism to sphere
    eps = 1e-8
    unit_vec = vec_prism_to_sphere / (dist_center_to_prism.unsqueeze(-1) + eps)  # (batch_size, 3, 1)

    # Closest point on sphere surface (toward prism)
    point_on_sphere = sphere_center - unit_vec * sphere_radius  # (batch_size, 3, 1)

    # Signed distance: distance from center to prism surface, minus sphere radius
    # Use sign based on whether sphere is inside or outside the prism
    sign = torch.where(is_inside.unsqueeze(-1),
                       -torch.ones_like(dist_center_to_prism),
                       torch.ones_like(dist_center_to_prism))
    signed_distance = sign * dist_center_to_prism - sphere_radius_raw.unsqueeze(-1)  # (batch_size, 1)

    # Ensure shape is (batch_size, 1)
    while len(signed_distance.shape) > 2:
        signed_distance = signed_distance.squeeze(-1)
    if len(signed_distance.shape) == 1:
        signed_distance = signed_distance.unsqueeze(-1)

    # Normals:
    # Sphere normal: outward from sphere center
    normal_on_sphere = -unit_vec  # Points from sphere toward prism, so negate for outward normal

    # Prism normal: depends on which face the closest point is on
    # In body frame, determine which axis is at the boundary
    eps_face = 1e-6
    at_x_face = torch.abs(torch.abs(closest_body_2d[:, 0:1]) - half_lens[:, 0:1]) < eps_face  # (batch_size, 1)
    at_y_face = torch.abs(torch.abs(closest_body_2d[:, 1:2]) - half_lens[:, 1:2]) < eps_face  # (batch_size, 1)
    at_z_face = torch.abs(torch.abs(closest_body_2d[:, 2:3]) - half_lens[:, 2:3]) < eps_face  # (batch_size, 1)

    # Normal in body frame - select the axis that's at the boundary
    # Use sign of the coordinate to determine direction
    normal_body_x = torch.sign(closest_body_2d[:, 0:1])  # (batch_size, 1)
    normal_body_y = torch.sign(closest_body_2d[:, 1:2])  # (batch_size, 1)
    normal_body_z = torch.sign(closest_body_2d[:, 2:3])  # (batch_size, 1)

    # Construct normal in body frame - prioritize the face with smallest distance
    normal_body_2d = torch.zeros_like(closest_body_2d)  # (batch_size, 3)
    # Set the component corresponding to the closest face
    normal_body_2d = torch.where(
        at_x_face,
        torch.cat([normal_body_x, torch.zeros_like(normal_body_y), torch.zeros_like(normal_body_z)], dim=1),
        normal_body_2d)
    normal_body_2d = torch.where(
        ~at_x_face & at_y_face,
        torch.cat([torch.zeros_like(normal_body_x), normal_body_y, torch.zeros_like(normal_body_z)], dim=1),
        normal_body_2d)
    normal_body_2d = torch.where(
        ~at_x_face & ~at_y_face & at_z_face,
        torch.cat([torch.zeros_like(normal_body_x), torch.zeros_like(normal_body_y), normal_body_z], dim=1),
        normal_body_2d)

    # If none are exactly at a face (edge/corner case), use the direction from prism to sphere
    at_any_face = at_x_face | at_y_face | at_z_face
    normal_body_2d = torch.where(at_any_face, normal_body_2d, unit_vec.squeeze(-1))

    # Transform normal back to world frame
    normal_body = normal_body_2d.unsqueeze(-1)  # (batch_size, 3, 1)
    normal_on_prism = torch.einsum("bij,bjk->bik", prism.rot_mat, normal_body)  # (batch_size, 3, 1)
    # Normalize to ensure unit length
    normal_on_prism = normal_on_prism / (torch.linalg.norm(normal_on_prism, dim=1, keepdim=True) + eps)

    return point_on_sphere, point_on_prism, signed_distance, normal_on_sphere, normal_on_prism


def cylinder_static_prism_signed_distance(
        cylinder: Cylinder,
        prism: StaticPrism,
        num_axis_samples: int = 10,
        num_circ_samples: int = 8
) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """
    Compute signed distance between a cylinder and a static prism (excluding endcaps).

    Args:
        cylinder: Cylinder object
        prism: StaticPrism object
        num_axis_samples: Number of samples along the cylinder axis (default: 20)
        num_circ_samples: Number of samples around the circumference (default: 20)

    Returns:
        point_on_cylinder: Closest point on cylinder surface (batch_size, 3, 1)
        point_on_prism: Closest point on prism surface (batch_size, 3, 1)
        signed_distance: Signed distance (batch_size, 1)
        normal_on_cylinder: Outward normal at point_on_cylinder (batch_size, 3, 1)
        normal_on_prism: Outward normal at point_on_prism (batch_size, 3, 1)
    """
    # Get cylinder end points and radius - keep shape (batch_size, 3, 1)
    end_pts = cylinder._compute_end_pts()
    end_pt1 = end_pts[0]  # (batch_size, 3, 1)
    end_pt2 = end_pts[1]  # (batch_size, 3, 1)
    # Get batch size first
    batch_size = end_pt1.shape[0]
    # Handle radius - may be (1, 1, 1) or (batch_size, 1, 1)
    cylinder_radius_raw = cylinder.radius
    if cylinder_radius_raw.shape[0] == 1 and batch_size > 1:
        # Expand to match batch size
        cylinder_radius = cylinder_radius_raw.expand(batch_size, -1, -1)  # (batch_size, 1, 1)
    else:
        cylinder_radius = cylinder_radius_raw  # (batch_size, 1, 1) or (1, 1, 1)

    # Cylinder axis
    axis = end_pt2 - end_pt1  # (batch_size, 3, 1)
    axis_length = torch.linalg.norm(axis, dim=1, keepdim=True)  # (batch_size, 1, 1)
    axis_length = axis_length.squeeze(-1)  # (batch_size, 1)
    axis_unit = axis / axis_length  # (batch_size, 3, 1)

    # Transform cylinder end points to prism's body frame
    prism_pos = prism.pos  # (batch_size, 3, 1)
    rot_mat_inv = prism.rot_mat.transpose(1, 2)  # (batch_size, 3, 3)

    end_pt1_body = torch.einsum("bij,bjk->bik", rot_mat_inv, end_pt1 - prism_pos)  # (batch_size, 3, 1)
    end_pt2_body = torch.einsum("bij,bjk->bik", rot_mat_inv, end_pt2 - prism_pos)  # (batch_size, 3, 1)
    axis_body = end_pt2_body - end_pt1_body  # (batch_size, 3, 1)

    # Get prism half lengths
    half_lens = prism.half_lens.squeeze(-1)  # (batch_size, 3)

    # Sample points on the cylinder surface
    # We'll sample along the axis (t parameter) and around the circumference (angle parameter)
    # Sample along axis (excluding endcaps: t in (0, 1))
    t_samples = torch.linspace(0.0, 1.0, num_axis_samples, device=end_pt1.device,
                               dtype=end_pt1.dtype)  # (num_axis_samples,)

    # Sample angles around circumference
    angles = torch.linspace(0, 2 * torch.pi, num_circ_samples + 1, device=end_pt1.device, dtype=end_pt1.dtype)[
             :-1]  # (num_circ_samples,)

    # Create coordinate system for cylinder cross-section
    # We need two perpendicular vectors to the axis
    axis_2d = axis.squeeze(-1)  # (batch_size, 3)
    axis_norm = torch.linalg.norm(axis_2d, dim=1, keepdim=True)  # (batch_size, 1)
    axis_unit_2d = axis_2d / axis_norm  # (batch_size, 3)

    # Find a vector perpendicular to the axis
    # Use a default direction and project it
    default_vec = torch.tensor([1.0, 0.0, 0.0], device=axis_2d.device, dtype=axis_2d.dtype).unsqueeze(0).expand(
        batch_size, -1)  # (batch_size, 3)

    # Project default_vec onto plane perpendicular to axis
    proj = default_vec - torch.sum(default_vec * axis_unit_2d, dim=1, keepdim=True) * axis_unit_2d  # (batch_size, 3)
    proj_norm = torch.linalg.norm(proj, dim=1, keepdim=True)  # (batch_size, 1)
    eps = 1e-8

    # If projection is too small, use a different default vector
    # NOTE: Always compute the fallback (no data-dependent `if`) for torch.compile compat.
    small_proj_mask = proj_norm.squeeze(-1) < eps
    default_vec2 = torch.tensor([0.0, 1.0, 0.0], device=axis_2d.device, dtype=axis_2d.dtype).unsqueeze(0).expand(
        batch_size, -1)
    proj2 = default_vec2 - torch.sum(default_vec2 * axis_unit_2d, dim=1, keepdim=True) * axis_unit_2d
    proj = torch.where(small_proj_mask.unsqueeze(-1), proj2, proj)
    proj_norm = torch.linalg.norm(proj, dim=1, keepdim=True)

    u_vec = proj / (proj_norm + eps)  # (batch_size, 3) - first perpendicular vector
    v_vec = torch.cross(axis_unit_2d, u_vec, dim=1)  # (batch_size, 3) - second perpendicular vector
    v_norm = torch.linalg.norm(v_vec, dim=1, keepdim=True)
    v_vec = v_vec / (v_norm + eps)

    # Generate sample points on cylinder surface using batched operations
    # Prepare radius value for broadcasting
    radius_val = cylinder_radius.squeeze(-1).squeeze(-1)  # (batch_size,) or scalar
    if len(radius_val.shape) == 0:
        radius_val = radius_val.unsqueeze(0).expand(batch_size)
    radius_val = radius_val.unsqueeze(-1).unsqueeze(-1)  # (batch_size, 1, 1) for broadcasting

    # Compute points on axis for all t values: (batch_size, 3, num_axis_samples)
    # t_samples: (num_axis_samples,), axis: (batch_size, 3, 1), end_pt1: (batch_size, 3, 1)
    t_samples_expanded = t_samples.unsqueeze(0).unsqueeze(0)  # (1, 1, num_axis_samples)
    points_on_axis = end_pt1 + t_samples_expanded * axis  # (batch_size, 3, num_axis_samples)
    points_on_axis_2d = points_on_axis  # (batch_size, 3, num_axis_samples)

    # Compute cos and sin for all angles: (num_circ_samples,)
    cos_angles = torch.cos(angles)  # (num_circ_samples,)
    sin_angles = torch.sin(angles)  # (num_circ_samples,)

    # Compute offsets for all angles using broadcasting
    # u_vec: (batch_size, 3), v_vec: (batch_size, 3)
    # cos_angles: (num_circ_samples,), sin_angles: (num_circ_samples,)
    # radius_val: (batch_size, 1, 1)

    # Reshape for broadcasting: u_vec and v_vec need to be (batch_size, 3, 1)
    # cos_angles and sin_angles need to be (1, 1, num_circ_samples)
    u_vec_expanded = u_vec.unsqueeze(-1)  # (batch_size, 3, 1)
    v_vec_expanded = v_vec.unsqueeze(-1)  # (batch_size, 3, 1)
    cos_angles_expanded = cos_angles.unsqueeze(0).unsqueeze(0)  # (1, 1, num_circ_samples)
    sin_angles_expanded = sin_angles.unsqueeze(0).unsqueeze(0)  # (1, 1, num_circ_samples)

    # Compute offset for all angles: (batch_size, 3, num_circ_samples)
    offset_per_angle = (
                                   cos_angles_expanded * u_vec_expanded + sin_angles_expanded * v_vec_expanded) * radius_val  # (batch_size, 3, num_circ_samples)

    # Combine points_on_axis (batch_size, 3, num_axis_samples) with offsets (batch_size, 3, num_circ_samples)
    # to get all combinations: (batch_size, 3, num_axis_samples, num_circ_samples)
    points_on_axis_expanded = points_on_axis_2d.unsqueeze(-1)  # (batch_size, 3, num_axis_samples, 1)
    offset_expanded = offset_per_angle.unsqueeze(2)  # (batch_size, 3, 1, num_circ_samples)
    surface_points_4d = points_on_axis_expanded + offset_expanded  # (batch_size, 3, num_axis_samples, num_circ_samples)

    # Reshape to (batch_size, 3, num_axis_samples * num_circ_samples)
    num_samples = num_axis_samples * num_circ_samples
    sample_points_world = surface_points_4d.reshape(batch_size, 3, num_samples)  # (batch_size, 3, num_samples)

    # Compute SDF for all sample points (SDF expects shape (batch_size, 3, num_points))
    sdf_values = prism.sdf(sample_points_world)  # (batch_size, num_samples)

    # Find the sample with minimum SDF
    min_idx = torch.argmin(sdf_values, dim=1)  # (batch_size,)
    batch_indices = torch.arange(batch_size, device=sample_points_world.device)

    # Get the best point on cylinder surface
    point_on_cylinder = sample_points_world[batch_indices, :, min_idx].unsqueeze(-1)  # (batch_size, 3, 1)

    # Find closest point on prism surface to point_on_cylinder
    # Transform to prism body frame
    point_on_cylinder_body = torch.einsum("bij,bjk->bik", rot_mat_inv,
                                          point_on_cylinder - prism_pos)  # (batch_size, 3, 1)
    point_on_cylinder_body_2d = point_on_cylinder_body.squeeze(-1)  # (batch_size, 3)

    # Clamp to box bounds (this gives closest point if outside)
    closest_body_2d = torch.clamp(point_on_cylinder_body_2d,
                                  min=-half_lens,
                                  max=half_lens)  # (batch_size, 3)

    # Check if inside and adjust
    dist_to_surface_body = torch.abs(point_on_cylinder_body_2d) - half_lens  # (batch_size, 3)
    max_dist = torch.max(dist_to_surface_body, dim=1, keepdim=True)[0]  # (batch_size, 1)
    is_inside = max_dist < 0

    # NOTE: Always compute the inside path (no data-dependent `if`) for torch.compile compat.
    # For points inside, find the closest face and vectorize the per-axis update.
    min_dist_idx = torch.argmin(torch.abs(dist_to_surface_body), dim=1)  # (batch_size,)
    one_hot = torch.nn.functional.one_hot(min_dist_idx, num_classes=3).bool()  # (batch_size, 3)

    # Where the closest-face coordinate matches, snap to the boundary; elsewhere keep original
    closest_body_inside_2d = torch.where(
        one_hot,
        torch.sign(point_on_cylinder_body_2d) * half_lens,
        point_on_cylinder_body_2d
    )  # (batch_size, 3)
    closest_body_2d = torch.where(is_inside, closest_body_inside_2d, closest_body_2d)

    closest_body = closest_body_2d.unsqueeze(-1)  # (batch_size, 3, 1)
    point_on_prism = torch.einsum("bij,bjk->bik", prism.rot_mat, closest_body) + prism_pos  # (batch_size, 3, 1)

    # Compute signed distance using SDF at the best point
    sdf_at_best = sdf_values[batch_indices, min_idx]  # (batch_size,)
    radius_val = cylinder_radius.squeeze(-1).squeeze(-1)  # (batch_size,) or scalar
    if len(radius_val.shape) == 0:
        radius_val = radius_val.unsqueeze(0).expand(batch_size)
    signed_distance = sdf_at_best.unsqueeze(-1) - radius_val.unsqueeze(-1)  # (batch_size, 1)

    # Normals:
    # Cylinder normal: perpendicular to axis, pointing from axis to surface point
    # Find the closest point on cylinder axis for the selected point_on_cylinder
    vec_from_end1 = point_on_cylinder - end_pt1  # (batch_size, 3, 1)
    proj_on_axis = torch.sum(vec_from_end1 * axis_unit, dim=1, keepdim=True)  # (batch_size, 1, 1)
    proj_on_axis = proj_on_axis.squeeze(-1)  # (batch_size, 1)
    proj_on_axis_clamped = torch.clamp(proj_on_axis, min=torch.zeros_like(axis_length), max=axis_length)
    closest_on_axis_for_point = end_pt1 + proj_on_axis_clamped.unsqueeze(-1) * axis_unit  # (batch_size, 3, 1)

    vec_axis_to_surface = point_on_cylinder - closest_on_axis_for_point  # (batch_size, 3, 1)
    dist_axis_to_surface = torch.linalg.norm(vec_axis_to_surface, dim=1, keepdim=True)  # (batch_size, 1, 1)
    normal_on_cylinder = vec_axis_to_surface / (dist_axis_to_surface + eps)  # (batch_size, 3, 1)

    # Prism normal: depends on which face the closest point is on
    eps_face = 1e-6
    at_x_face = torch.abs(torch.abs(closest_body_2d[:, 0:1]) - half_lens[:, 0:1]) < eps_face  # (batch_size, 1)
    at_y_face = torch.abs(torch.abs(closest_body_2d[:, 1:2]) - half_lens[:, 1:2]) < eps_face  # (batch_size, 1)
    at_z_face = torch.abs(torch.abs(closest_body_2d[:, 2:3]) - half_lens[:, 2:3]) < eps_face  # (batch_size, 1)

    # Normal in body frame
    normal_body_x = torch.sign(closest_body_2d[:, 0:1])  # (batch_size, 1)
    normal_body_y = torch.sign(closest_body_2d[:, 1:2])  # (batch_size, 1)
    normal_body_z = torch.sign(closest_body_2d[:, 2:3])  # (batch_size, 1)

    # Construct normal in body frame
    normal_body_2d = torch.zeros_like(closest_body_2d)  # (batch_size, 3)
    normal_body_2d = torch.where(at_x_face,
                                 torch.cat(
                                     [normal_body_x, torch.zeros_like(normal_body_y), torch.zeros_like(normal_body_z)],
                                     dim=1),
                                 normal_body_2d)
    normal_body_2d = torch.where(~at_x_face & at_y_face,
                                 torch.cat(
                                     [torch.zeros_like(normal_body_x), normal_body_y, torch.zeros_like(normal_body_z)],
                                     dim=1),
                                 normal_body_2d)
    normal_body_2d = torch.where(~at_x_face & ~at_y_face & at_z_face,
                                 torch.cat(
                                     [torch.zeros_like(normal_body_x), torch.zeros_like(normal_body_y), normal_body_z],
                                     dim=1),
                                 normal_body_2d)

    # If none are exactly at a face (edge/corner case), use the direction from prism to cylinder
    at_any_face = at_x_face | at_y_face | at_z_face
    vec_prism_to_cyl = point_on_cylinder - point_on_prism  # (batch_size, 3, 1)
    vec_prism_to_cyl_normalized = vec_prism_to_cyl / (torch.linalg.norm(vec_prism_to_cyl, dim=1, keepdim=True) + eps)
    normal_body_2d = torch.where(at_any_face, normal_body_2d, vec_prism_to_cyl_normalized.squeeze(-1))

    # Transform normal back to world frame
    normal_body = normal_body_2d.unsqueeze(-1)  # (batch_size, 3, 1)
    normal_on_prism = torch.einsum("bij,bjk->bik", prism.rot_mat, normal_body)  # (batch_size, 3, 1)
    # Normalize to ensure unit length
    normal_on_prism = normal_on_prism / (torch.linalg.norm(normal_on_prism, dim=1, keepdim=True) + eps)

    return point_on_cylinder, point_on_prism, signed_distance, normal_on_cylinder, normal_on_prism


def sphere_static_rect_plane_signed_distance(
        sphere: SphereState,
        plane: StaticRectPlane
) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """
    Compute signed distance between a sphere and a static rectangular plane.

    Args:
        sphere: Sphere object
        plane: StaticRectPlane object

    Returns:
        point_on_sphere: Closest point on sphere surface (batch_size, 3, 1)
        point_on_plane: Closest point on plane surface (batch_size, 3, 1)
        signed_distance: Signed distance (batch_size, 1)
        normal_on_sphere: Outward normal at point_on_sphere (batch_size, 3, 1)
        normal_on_plane: Outward normal at point_on_plane (batch_size, 3, 1)
    """
    # Get sphere center - keep shape (batch_size, 3, 1)
    sphere_center = sphere.pos  # (batch_size, 3, 1)
    batch_size = sphere_center.shape[0]

    # Get sphere radius
    sphere_radius_raw = sphere.radius.squeeze(-1).squeeze(-1)  # (batch_size,) or scalar
    if len(sphere_radius_raw.shape) == 0:
        sphere_radius_raw = sphere_radius_raw.unsqueeze(0)
    if sphere_radius_raw.shape[0] == 1 and batch_size > 1:
        sphere_radius_raw = sphere_radius_raw.expand(batch_size)
    sphere_radius = sphere_radius_raw.unsqueeze(-1).unsqueeze(-1)  # (batch_size, 1, 1)

    # Transform sphere center to plane's local coordinate system
    rel_pt = sphere_center - plane.pos  # (batch_size, 3, 1)

    # Project onto local axes to get coordinates in plane's frame
    x_local = torch.einsum('bij,bjk->bik', plane.x_axis.transpose(1, 2), rel_pt)  # (batch_size, 1, 1)
    y_local = torch.einsum('bij,bjk->bik', plane.y_axis.transpose(1, 2), rel_pt)  # (batch_size, 1, 1)
    z_local = torch.einsum('bij,bjk->bik', plane.z_axis.transpose(1, 2), rel_pt)  # (batch_size, 1, 1)

    # Extract half-lengths (ensure proper broadcasting shape)
    half_x = plane.half_lens[0].reshape(1, 1, 1) if plane.half_lens[0].ndim == 0 else plane.half_lens[0]
    half_y = plane.half_lens[1].reshape(1, 1, 1) if plane.half_lens[1].ndim == 0 else plane.half_lens[1]

    # Find closest point on the finite rectangular plane
    # Clamp the x and y coordinates to the rectangle bounds
    x_clamped = torch.clamp(x_local, min=-half_x, max=half_x)  # (batch_size, 1, 1)
    y_clamped = torch.clamp(y_local, min=-half_y, max=half_y)  # (batch_size, 1, 1)
    # The z coordinate on the plane is always 0 in the local frame
    z_on_plane = torch.zeros_like(z_local)  # (batch_size, 1, 1)

    # Transform closest point back to world coordinates
    # point = plane.pos + x_clamped * x_axis + y_clamped * y_axis + z_on_plane * z_axis
    point_on_plane = (plane.pos +
                      x_clamped * plane.x_axis +
                      y_clamped * plane.y_axis +
                      z_on_plane * plane.z_axis)  # (batch_size, 3, 1)

    # Vector from closest point on plane to sphere center
    vec_plane_to_sphere = sphere_center - point_on_plane  # (batch_size, 3, 1)
    dist_center_to_plane = torch.linalg.norm(vec_plane_to_sphere, dim=1, keepdim=True)  # (batch_size, 1, 1)
    dist_center_to_plane = dist_center_to_plane.squeeze(-1)  # (batch_size, 1)

    # Unit vector from plane to sphere
    eps = 1e-8
    unit_vec = vec_plane_to_sphere / (dist_center_to_plane.unsqueeze(-1) + eps)  # (batch_size, 3, 1)

    # Closest point on sphere surface (toward plane)
    point_on_sphere = sphere_center - unit_vec * sphere_radius  # (batch_size, 3, 1)

    # Signed distance: the sphere intersects the finite plane when its closest point on the plane
    # is within sphere_radius of the sphere center. dist_center_to_plane is already the distance
    # to the clamped (finite) closest point, so this is the correct geometric intersection test.
    dist_between_points = torch.linalg.norm(point_on_sphere - point_on_plane, dim=1, keepdim=True)  # (batch_size, 1, 1)
    dist_between_points = dist_between_points.squeeze(-1)  # (batch_size, 1)

    is_penetrating = dist_center_to_plane < sphere_radius_raw.unsqueeze(-1)  # (batch_size, 1)

    signed_distance = torch.where(
        is_penetrating,
        -dist_between_points,  # Negative penetration
        dist_between_points    # Positive separation
    )

    # Ensure shape is (batch_size, 1)
    while len(signed_distance.shape) > 2:
        signed_distance = signed_distance.squeeze(-1)
    if len(signed_distance.shape) == 1:
        signed_distance = signed_distance.unsqueeze(-1)

    # Normals:
    # Sphere normal: outward from sphere center
    normal_on_sphere = -unit_vec  # Points from sphere toward plane, so negate for outward normal

    # Plane normal: z_axis direction, pointing toward the sphere center.
    # z_local is the signed projection of the sphere center onto the plane normal;
    # positive means the center is on the normal side, negative means it's behind the plane.
    z_local_2d = z_local.squeeze(-1)  # (batch_size, 1)
    normal_sign = torch.sign(z_local_2d)  # (batch_size, 1)
    # Default to positive normal side when sphere center lies exactly on the plane.
    normal_sign = torch.where(normal_sign == 0, torch.ones_like(normal_sign), normal_sign)
    normal_on_plane = normal_sign.unsqueeze(-1) * plane.z_axis  # (batch_size, 3, 1)

    return point_on_sphere, point_on_plane, signed_distance, normal_on_sphere, normal_on_plane


def get_dist_fn(obj1: RigidBody, obj2: RigidBody) -> Callable:
    """
    Get the signed distance function for two rigid bodies.

    Args:
        obj1: First rigid body object
        obj2: Second rigid body object

    Returns:
        dist_fn: Signed distance function
    """
    if isinstance(obj1, SphereState) and isinstance(obj2, SphereState):
        return sphere_sphere_signed_distance
    elif isinstance(obj1, SphereState) and isinstance(obj2, Cylinder):
        return sphere_cylinder_signed_distance
    elif isinstance(obj1, Cylinder) and isinstance(obj2, Cylinder):
        return cylinder_cylinder_signed_distance
    elif isinstance(obj1, SphereState) and isinstance(obj2, StaticPrism):
        return sphere_static_prism_signed_distance
    elif isinstance(obj1, Cylinder) and isinstance(obj2, StaticPrism):
        return cylinder_static_prism_signed_distance
    elif isinstance(obj1, SphereState) and isinstance(obj2, StaticRectPlane):
        return sphere_static_rect_plane_signed_distance
    else:
        raise ValueError(f"Unsupported combination of objects: {type(obj1)} and {type(obj2)}")

    return dist_fn
