from typing import List, Optional, Dict, Tuple, Union

import torch

from gnn_simulator.state_objects.rigid_object import RigidBody
from gnn_simulator.utilities import torch_quaternion
from gnn_simulator.utilities.misc_utils import DEFAULT_DTYPE
from gnn_simulator.utilities.inertia_tensors import cylinder_body, hollow_cylinder_body, solid_sphere_body, \
    rect_prism_body


class Cylinder(RigidBody):

    def __init__(self,
                 name: str,
                 end_pts: Union[torch.Tensor, List],
                 linear_vel: torch.Tensor,
                 ang_vel: torch.Tensor,
                 radius: torch.Tensor,
                 mass: torch.Tensor,
                 sites: Dict):
        self.shape = 'cylinder'

        self._body_verts = None
        self._faces = None

        self.end_pts = end_pts
        if isinstance(end_pts, torch.Tensor):
            self.end_pts = end_pts.reshape(-1, 3, 2)
            self.end_pts = [self.end_pts[:, :, :1], self.end_pts[:, :, 1:2]]

        self.radius = radius
        self.length = (self.end_pts[0] - self.end_pts[1]).norm(dim=1).squeeze()  # compute length

        linear_vel = linear_vel.reshape(-1, 3, 1)
        pos = (self.end_pts[0] + self.end_pts[1]) / 2.0  # compute pos from end points
        ang_vel = ang_vel.reshape(-1, 3, 1)

        # Compute an initial quaternion and rotation matrix
        prin = self.end_pts[1] - self.end_pts[0]
        q = torch_quaternion.compute_quat_btwn_z_and_vec(prin)

        I_body = cylinder_body(mass, self.length, self.radius, DEFAULT_DTYPE)

        super().__init__(name,
                         mass,
                         I_body,
                         pos,
                         q,
                         linear_vel,
                         ang_vel,
                         sites)

    def to(self, device):
        super(Cylinder, self).to(device)
        self.radius = self.radius.to(device)
        self.length = self.length.to(device)

        self.end_pts[0] = self.end_pts[0].to(device)
        self.end_pts[1] = self.end_pts[1].to(device)

        return self

    def get_principal_axis(self):
        """
        Method to get principal axis
        :return:
        """
        # z-axis aligned with rod cylinder axis along length
        return self.rot_mat[..., :, 2:]

    def _compute_end_pts(self) -> List[torch.Tensor]:
        """
        Internal method to compute end points
        :return: End point tensors
        """
        end_pts = self.compute_end_pts_from_state(
            self.state,
            self.length
        )
        # end_pts = torch.concat(end_pts, dim=-1)

        return end_pts

    @staticmethod
    def compute_principal_axis(quat):
        """
        Computes principal axis from state input

        :param state: State (pos1, pos2, pos3, q1, q2, q3, q4, lin_v1, lin_v2, lin_v3, ang_v1, ang_v2, ang_v3)
        :return: principal axises
        """
        return torch_quaternion.quat_as_rot_mat(quat)[..., :, 2:]

    @staticmethod
    def compute_end_pts_from_state(rod_pose, rod_length):
        """
        :param rod_pose: (x, y, z, quat.w, quat.x, quat.y, quat.z)
        :param rod_length: length of rod
        :return: ((x1, y1, z1), (x2, y2, z2))
        """
        # Get position
        pos = rod_pose[:, :3]
        quat = rod_pose[:, 3:]
        prin_axis = torch_quaternion.compute_prin_axis(quat)

        # Compute half-length vector from principal axis
        half_length_vec = rod_length * prin_axis / 2

        # End points are +/- of half-length vector from COM
        end_pt1 = pos - half_length_vec
        end_pt2 = pos + half_length_vec

        return [end_pt1, end_pt2]

    def update_state(self, pos, linear_vel, rot_val, ang_vel):
        super().update_state(pos, linear_vel, rot_val, ang_vel)
        self.end_pts = self._compute_end_pts()

    def get_normal(self, pt):
        """
        Get normal vectors at surface points on the cylinder (curved surface only).

        :param pt: Batched tensor of points on the surface, shape (batch_size, 3, num_pts)
        :return: Normal vectors at the points, shape (batch_size, 3, num_pts)
        """
        # Get principal axis (z-axis in world frame)
        prin_axis = self.get_principal_axis()  # (batch_size, 3, 1)

        # Get relative position from center
        rel_pt = pt - self.pos  # (batch_size, 3, num_pts)

        # Project relative position onto principal axis
        proj_len = torch.einsum('bij,bjk->bik', prin_axis.transpose(1, 2), rel_pt)  # (batch_size, 1, num_pts)
        proj_vec = prin_axis * proj_len  # projection of rel_pt onto principal axis

        # Normal is perpendicular to principal axis (radial direction)
        radial_vec = rel_pt - proj_vec  # perpendicular component
        normal = radial_vec / (torch.linalg.norm(radial_vec, dim=1, keepdim=True) + 1e-8)

        return normal


class HollowCylinder(Cylinder):
    def __init__(self,
                 name,
                 end_pts,
                 linear_vel,
                 ang_vel,
                 outer_radius,
                 inner_radius,
                 mass,
                 sites):
        self.shape = 'cylinder'

        super().__init__(name,
                         end_pts,
                         linear_vel,
                         ang_vel,
                         outer_radius,
                         mass,
                         sites)

        self.inner_radius = inner_radius

        self.I_body = hollow_cylinder_body(mass,
                                           self.length,
                                           outer_radius,
                                           inner_radius)

        # self.I_body_inv = torch.linalg.inv(self.I_body)


class SphereState(RigidBody):

    def __init__(self,
                 name: str,
                 center: torch.Tensor,
                 linear_vel: Optional[torch.Tensor],
                 ang_vel: Optional[torch.Tensor],
                 radius: torch.Tensor,
                 mass: torch.Tensor,
                 principal_axis: Optional[torch.Tensor],
                 sites: Dict,
                 rot_val: Optional[torch.Tensor] = None):
        self.shape = 'sphere'

        self.radius = radius

        linear_vel = linear_vel.reshape(-1, 3, 1)
        ang_vel = ang_vel.reshape(-1, 3, 1)

        if rot_val is None:
            rot_val = torch_quaternion.compute_quat_btwn_z_and_vec(principal_axis)

        self._body_verts = None
        self._faces = None

        super().__init__(name,
                         mass,
                         solid_sphere_body(mass, self.radius, DEFAULT_DTYPE),
                         center,
                         rot_val,
                         linear_vel,
                         ang_vel,
                         sites)

    def signed_dist_fn(self, pt):
        if len(pt.shape) == 2:
            pt = pt.unsqueeze(-1)

        rel_pt = pt - self.pos
        sdf = rel_pt.norm(dim=1, keepdim=True) - self.radius

        return sdf

    def proj_surface_pt(self, pt):
        rel_pt = pt - self.pos
        dir_vec = rel_pt / torch.linalg.norm(rel_pt, dim=1)
        surface_pt = self.pos + dir_vec * self.radius

        return surface_pt

    def to(self, device):
        super(SphereState, self).to(device)
        self.radius = self.radius.to(device)

        if self._body_verts is not None:
            self._body_verts = self._body_verts.to(device)
            self._faces = self._faces.to(device)

        return self

    def get_normal(self, pt):
        """
        Get normal vectors at surface points on the sphere.

        :param pt: Batched tensor of points on the surface, shape (batch_size, 3, num_pts)
        :return: Normal vectors at the points, shape (batch_size, 3, num_pts)
        """
        # Normal is the direction from center to point
        rel_pt = pt - self.pos
        normal = rel_pt / (torch.linalg.norm(rel_pt, dim=1, keepdim=True) + 1e-8)

        return normal


class RectPrism(RigidBody):

    def __init__(self,
                 name: str,
                 center: torch.Tensor,
                 linear_vel: Optional[torch.Tensor],
                 ang_vel: Optional[torch.Tensor],
                 half_lens: Tuple[torch.Tensor],
                 mass: torch.Tensor,
                 rot_mat: torch.Tensor,
                 sites: Dict):
        quat = self.rot_mat_to_quat(rot_mat)
        self.half_lens = torch.cat(half_lens).reshape(1, 3, 1)

        inertia_tensor = rect_prism_body(mass,
                                         2 * self.half_lens[0, 0].item(),
                                         2 * self.half_lens[0, 1].item(),
                                         2 * self.half_lens[0, 2].item(),
                                         DEFAULT_DTYPE)
        super().__init__(name,
                         mass,
                         inertia_tensor,
                         center,
                         quat,
                         linear_vel.reshape(-1, 3, 1),
                         ang_vel.reshape(-1, 3, 1),
                         sites)

    def to(self, device):
        super(RectPrism, self).to(device)
        self.half_lens = self.half_lens.to(device)

        return self
    
    @staticmethod
    def rot_mat_to_quat(rot_mat):
        """
        rot_mat: (1, 3, 3)
        """
        if len(rot_mat.shape) == 2:
            rot_mat = rot_mat.unsqueeze(0)

        assert rot_mat.shape[1:] == (3, 3)

        q_z = torch_quaternion.compute_quat_btwn_z_and_vec(rot_mat[..., 2:])
        x_p = torch_quaternion.rotate_vec_quat(torch_quaternion.inverse_unit_quat(q_z), rot_mat[..., :1])

        # Use tensors directly, not .item() - atan2 expects tensors
        angle = torch.atan2(x_p[0, 1, 0], x_p[0, 0, 0]) / 2.
        q_xy = torch.tensor(
            [torch.cos(angle), 0., 0., torch.sin(angle)],
            dtype=rot_mat.dtype,
            device=rot_mat.device
        ).reshape(1, 4, 1)

        q_final = torch_quaternion.quat_prod(q_z, q_xy)

        return q_final

    def sdf(self, pt):
        # pt: (batch_size, 3, num_pts)
        # self.rot_mat: (batch_size, 3, 3)
        # self.pos: (batch_size, 3, 1)
        # self.half_lens: (1, 3, 1)

        # Transform points to body frame: (batch_size, 3, num_pts)
        pt_rel = torch.einsum("bij,bjk->bik", self.rot_mat.transpose(1, 2), pt - self.pos)

        # Compute distance to box faces: (batch_size, 3, num_pts)
        d = torch.abs(pt_rel) - self.half_lens

        # Outside distance: (batch_size, num_pts)
        outside = torch.clamp(d, min=0)
        outside_dist = torch.linalg.norm(outside, dim=1)

        # Inside distance: (batch_size, num_pts)
        inside_dist = torch.minimum(
            d.max(dim=1).values,
            torch.zeros_like(outside_dist)
        )

        # Final signed distance: (batch_size, num_pts)
        dist = outside_dist + inside_dist

        return dist

    def get_normal(self, pt):
        """
        Get normal vectors at surface points on the rectangular prism.

        :param pt: Batched tensor of points on the surface, shape (batch_size, 3, num_pts)
        :return: Normal vectors at the points, shape (batch_size, 3, num_pts)
        """
        # Transform points to body frame
        pt_rel = torch.einsum("bij,bjk->bik", self.rot_mat.transpose(1, 2), pt - self.pos)

        # Compute distance to each face
        d = torch.abs(pt_rel) - self.half_lens  # (batch_size, 3, num_pts)

        # The face with d closest to 0 is the one the point is on
        # Find which axis the point is closest to the boundary
        abs_d = torch.abs(d)
        min_idx = torch.argmin(abs_d, dim=1, keepdim=True)  # (batch_size, 1, num_pts)

        # Create one-hot encoding for the axis
        one_hot = torch.zeros_like(pt_rel)
        one_hot.scatter_(1, min_idx, 1.0)

        # Normal in body frame (sign indicates direction)
        normal_body = one_hot * torch.sign(pt_rel)

        # Transform back to world frame
        normal_world = torch.einsum("bij,bjk->bik", self.rot_mat, normal_body)

        return normal_world


class StaticPrism(RectPrism):

    def __init__(self,
                 name: str,
                 center: torch.Tensor,
                 rot_mat: torch.Tensor,
                 half_lens: Tuple[torch.Tensor, torch.Tensor, torch.Tensor],
                 dtype=DEFAULT_DTYPE):
        mass = torch.tensor(torch.inf, dtype=dtype).reshape(1, 1, 1)
        lin_vel, ang_vel = torch.zeros_like(center), torch.zeros_like(center)

        super().__init__(name,
                         center,
                         lin_vel,
                         ang_vel,
                         half_lens,
                         mass,
                         rot_mat,
                         [])
    
    def repeat_state(self, batch_size):
        self.reset_batch_size()

        self.pos = self.pos.repeat(batch_size, 1, 1)
        self.quat = self.quat.repeat(batch_size, 1, 1)
        self.linear_vel = self.linear_vel.repeat(batch_size, 1, 1)
        self.ang_vel = self.ang_vel.repeat(batch_size, 1, 1)

    def reset_batch_size(self):
        self.pos = self.pos[:1]
        self.quat = self.quat[:1]
        self.linear_vel = self.linear_vel[:1]
        self.ang_vel = self.ang_vel[:1]


class StaticRectPlane(RigidBody):

    def __init__(self,
                 name: str,
                 center: torch.Tensor,
                 rot_mat: torch.Tensor,
                 half_lens: Tuple[torch.Tensor, torch.Tensor],
                 dtype=DEFAULT_DTYPE):
        """
        :param name: name of the plane
        :param center: center of the plane
        :param rot_mat: rotation matrix of the plane
        :param half_lens: half lengths of the plane
        :param dtype: data type
        """
        mass = torch.tensor(torch.inf, dtype=dtype).reshape(1, 1, 1)
        I_body = torch.diag(torch.tensor([torch.inf] * 3, dtype=dtype)).reshape(1, 3, 3)
        quat = RectPrism.rot_mat_to_quat(rot_mat)
        linear_vel = torch.zeros_like(center)
        ang_vel = torch.zeros_like(center)

        self.x_axis = rot_mat[..., :1].reshape(1, 3, 1)
        self.y_axis = rot_mat[..., 1:2].reshape(1, 3, 1)
        self.z_axis = rot_mat[..., 2:].reshape(1, 3, 1)
        self.half_lens = half_lens

        super().__init__(name, mass, I_body, center, quat, linear_vel, ang_vel, [])

    def to(self, device):
        super().to(device)
        self.x_axis = self.x_axis.to(device)
        self.y_axis = self.y_axis.to(device)
        self.z_axis = self.z_axis.to(device)
        self.half_lens = tuple(h.to(device) for h in self.half_lens)

        return self

    def update_state(self, pos, linear_vel, quat, ang_vel):
        super().update_state(pos, linear_vel, quat, ang_vel)
        rot_mat = self.rot_mat
        self.x_axis = rot_mat[..., :1]
        self.y_axis = rot_mat[..., 1:2]
        self.z_axis = rot_mat[..., 2:]
        
    def sdf(self, pt):
        """
        Compute signed distance from point(s) to the finite rectangular plane.

        Points in the positive normal direction (self.z_axis) have positive distances.
        Points in the negative normal direction have negative distances.

        :param pt: Points to evaluate, shape (batch_size, 3, num_pts) or (batch_size, 3)
        :return: Signed distances, shape (batch_size, 1, num_pts)
        """
        # Handle input shape
        if len(pt.shape) == 2:
            pt = pt.unsqueeze(-1)

        # Transform point to plane's local coordinate system
        rel_pt = pt - self.pos  # (batch_size, 3, num_pts)

        # Project onto local axes to get coordinates in plane's frame
        x_local = torch.einsum('bij,bjk->bik', self.x_axis.transpose(1, 2), rel_pt)  # (batch_size, 1, num_pts)
        y_local = torch.einsum('bij,bjk->bik', self.y_axis.transpose(1, 2), rel_pt)  # (batch_size, 1, num_pts)
        z_local = torch.einsum('bij,bjk->bik', self.z_axis.transpose(1, 2), rel_pt)  # (batch_size, 1, num_pts)

        # Extract half-lengths (ensure proper broadcasting shape)
        half_x = self.half_lens[0].reshape(1, 1, 1) if self.half_lens[0].ndim == 0 else self.half_lens[0]
        half_y = self.half_lens[1].reshape(1, 1, 1) if self.half_lens[1].ndim == 0 else self.half_lens[1]

        # Compute clamped distance in tangent plane (x-y plane)
        # Distance is 0 if within bounds, positive if outside
        dx = torch.clamp(torch.abs(x_local) - half_x, min=0.0)
        dy = torch.clamp(torch.abs(y_local) - half_y, min=0.0)

        # Tangent plane distance (horizontal distance to rectangle boundary)
        tangent_dist_sq = dx**2 + dy**2

        # Signed distance is the Euclidean distance to nearest point on rectangle,
        # with sign determined by which side of the plane the point is on
        sdf = torch.sign(z_local) * torch.sqrt(tangent_dist_sq + z_local**2)

        return sdf

    def get_normal(self, pt):
        """
        Get normal vectors at surface points on the rectangular plane.
        """
        return self.z_axis.repeat(pt.shape[0], 1, 1)

    def repeat_state(self, batch_size):
        self.reset_batch_size()

        self.pos = self.pos.repeat(batch_size, 1, 1)
        self.quat = self.quat.repeat(batch_size, 1, 1)
        self.linear_vel = self.linear_vel.repeat(batch_size, 1, 1)
        self.ang_vel = self.ang_vel.repeat(batch_size, 1, 1)

    def reset_batch_size(self):
        self.pos = self.pos[:1]
        self.quat = self.quat[:1]
        self.linear_vel = self.linear_vel[:1]
        self.ang_vel = self.ang_vel[:1]


class FlatGround(StaticRectPlane):

    def __init__(self, 
                 name: str = "ground",
                 center: torch.Tensor | None = None,
                 rot_mat: torch.Tensor | None = None,
                 half_lens: Tuple[torch.Tensor, torch.Tensor] | None = None,
                 dtype=DEFAULT_DTYPE):
        if center is None:
            center = torch.zeros((1, 3, 1), dtype=dtype)
        if rot_mat is None:
            rot_mat = torch.eye(3, dtype=dtype).unsqueeze(0)
        if half_lens is None:
            half_lens = (torch.tensor(100., dtype=dtype), torch.tensor(100., dtype=dtype))
        super().__init__(name, center, rot_mat, half_lens, dtype)
