from typing import Tuple, Union, Dict

import torch

from gnn_simulator.linear_contact.collision_detector import get_detector
from gnn_simulator.linear_contact.collision_response import CollisionResponseGenerator
from gnn_simulator.simulators.abstract_simulator import AbstractPhysicsSimulator
from gnn_simulator.state_objects.rigid_object import RigidBody
from gnn_simulator.utilities import torch_quaternion


class RigidBodySimulator(AbstractPhysicsSimulator):
    """
    Class for rod simulations
    """

    def __init__(self,
                 rigid_body: RigidBody,
                 gravity: torch.Tensor,
                 contact_params: Dict[str, Dict[str, torch.Tensor]] = None,
                 use_contact: bool = True):
        """
        :param rigid_body: RodState objects
        :param gravity: Gravity constant
        """
        super().__init__(rigid_body)

        self.gravity = gravity

        if use_contact:
            self.collision_resp_gen = CollisionResponseGenerator()
            self.collision_resp_gen.set_contact_params('default', contact_params)
        else:
            self.collision_resp_gen = None

    def to(self, device):
        self.robot.to(device)
        self.collision_resp_gen.to(device)
        self.gravity = self.gravity.to(device)

        return self

    def compute_forces(self, external_forces, external_pts) \
            -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        See parent documentation.
        """
        if len(external_forces.shape) == 2:
            external_forces = external_forces.reshape(1, 3, -1)
            external_pts = external_pts.reshape(1, 3, -1)

        gravity_force = self.robot.compute_gravity_force(
            self.gravity,
            external_forces.shape[0]
        )

        forces = torch.concat([gravity_force, external_forces], dim=-1)
        acting_pts = torch.concat([self.robot.pos, external_pts], dim=-1)

        net_force = forces.sum(dim=-1, keepdim=True)

        return net_force, forces, acting_pts

    def compute_accelerations(self, net_force, net_torque):
        """
        Compute rigid-body accelerations
        See parent documentation
        """

        # Linear acceleration of rigid body = sum(Forces) / mass
        lin_acc = net_force / self.robot.mass

        # Angular acceleration of rigid body = (I^-1) * sum(Torques)
        ang_acc = self.robot.I_world_inv @ net_torque

        return lin_acc, ang_acc

    def time_integration(self, lin_acc, ang_acc, dt):
        """
        Semi-implicit euler

        See parent documentation.
        """
        # Semi-explicit euler step for velocity and position
        linear_vel = self.robot.linear_vel + lin_acc * dt
        pos = self.robot.pos + linear_vel * dt

        # Semi-explicit euler step for angular velocity
        ang_vel = self.robot.ang_vel + ang_acc * dt

        # Angular velocity in quaternion format, semi-explicit euler on quaternion
        quat = self.update_quat(self.robot.quat, ang_vel, dt)

        # Concat position, quaternion, linear velocity, and angular velocity to form next state
        next_state = torch.concat([pos, quat, linear_vel, ang_vel], dim=1)

        return next_state

    def compute_contact_deltas(self,
                               pre_next_state: torch.Tensor,
                               dt: Union[torch.Tensor, float]
                               ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        detector = get_detector(self.robot, self.collision_resp_gen.ground)
        _, _, delta_v, delta_w, toi = (
            self.collision_resp_gen.resolve_contact_ground(
                self.robot,
                pre_next_state,
                dt,
                detector)
        )

        return delta_v, delta_w, toi

    def resolve_contacts(self,
                         pre_next_state: torch.Tensor,
                         dt: Union[torch.Tensor, float],
                         delta_v,
                         delta_w,
                         toi) -> torch.Tensor:
        lin_vel = pre_next_state[:, 7:10, ...] + delta_v
        ang_vel = pre_next_state[:, 10:, ...] + delta_w
        pos = self.robot.pos + dt * lin_vel - delta_v * toi

        quat = torch_quaternion.update_quat(self.robot.quat, ang_vel, dt)
        quat = torch_quaternion.update_quat(quat, -delta_w, toi)

        next_state = torch.hstack([pos, quat, lin_vel, ang_vel])

        return next_state

    def update_state(self, next_state: torch.Tensor) -> None:
        """
        Update state and other internal attribute from state
        """
        if len(next_state.shape) == 1:
            next_state = next_state.unsqueeze(0)

        next_state = next_state.reshape(-1, next_state.shape[1], 1)
        pos = next_state[:, :3]
        quat = next_state[:, 3:7]
        linear_vel = next_state[:, 7:10]
        ang_vel = next_state[:, 10:]

        self.robot.update_state(pos, linear_vel, quat, ang_vel)

    def get_xyz_pos(self, curr_state: torch.Tensor) -> torch.Tensor:
        """
        See parent documentation
        """
        return self.robot.pos

    def get_curr_state(self) -> torch.Tensor:
        """
        See parent documentation.
        """
        return self.robot.state
