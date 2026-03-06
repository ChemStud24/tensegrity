import json
import logging
import random
from copy import deepcopy
from typing import Dict, Union, List

import torch
import tqdm
from torch.nn.modules.loss import _Loss
from torch.utils.data import DataLoader

from gnn_simulator.nn_training.datasets.tensegrity_dataset import PlanarEnvGroupDataset
from gnn_simulator.nn_training.tensegrity_gnn_training_engine import (
    TensegrityMultiSimMultiStepMotorGNNTrainingEngine,
)
from gnn_simulator.simulators.planar_env_tensegrity_gnn_simulator import (
    PlanarEnvTensegrityGNNSimulator,
)
from gnn_simulator.state_objects.primitive_shapes import StaticRectPlane
from gnn_simulator.utilities import torch_quaternion


class CombinedGroupDataLoader:
    """Iterates over batches from multiple per-group DataLoaders.

    Collects all batches from every DataLoader, then optionally shuffles the
    combined list so groups are interleaved.  Each individual batch always
    comes entirely from one group, satisfying the same-dataset-per-batch
    requirement.
    """

    def __init__(self, dataloaders, shuffle=True):
        self.dataloaders = dataloaders
        self.shuffle = shuffle

    def __iter__(self):
        all_batches = [batch for dl in self.dataloaders for batch in dl]
        if self.shuffle:
            random.shuffle(all_batches)
        return iter(all_batches)

    def __len__(self):
        return sum(len(dl) for dl in self.dataloaders)


class PlanarEnvTensegrityMultiSimMultiStepMotorGNNTrainingEngine(
    TensegrityMultiSimMultiStepMotorGNNTrainingEngine
):

    def __init__(self,
                 training_config: Dict,
                 criterion: _Loss,
                 dt: Union[float, torch.Tensor],
                 logger: logging.Logger):

        is_multi_group = 'train_data_groups' in training_config

        if is_multi_group:
            self._train_data_groups = training_config['train_data_groups']
            self._val_data_groups   = training_config['val_data_groups']
            self._sim_config_paths  = training_config['sim_configs']

            # Build a merged config that is backward-compatible with the parent:
            # - use the first sim_config for GNN architecture / dummy simulator
            # - flatten all group data_paths (in group order) for data loading
            merged = dict(training_config)
            merged['sim_config'] = self._sim_config_paths[0]
            merged['train_data_paths'] = [
                p for g in self._train_data_groups for p in g['data_paths']
            ]
            merged['val_data_paths'] = [
                p for g in self._val_data_groups for p in g['data_paths']
            ]

            # Parent creates self.train_data_dict / self.val_data_dict (merged),
            # self.train_dataloader / self.val_dataloader (single-dataset), and
            # self.simulator (with the first sim_config's env objects).
            # NOTE: _save_env_obj_originals() is NOT called inside the parent
            # chain — it is deferred until all env objects are aggregated below.
            super().__init__(merged, criterion, dt, logger)

            # 1. Tag each trajectory in the merged data_dicts with its group id.
            #    This is consumed by evaluate_rollouts for per-trajectory env swapping.
            self.train_data_dict['env_group_ids'] = [
                i for i, g in enumerate(self._train_data_groups)
                for _ in g['data_paths']
            ]
            self.val_data_dict['env_group_ids'] = [
                i for i, g in enumerate(self._val_data_groups)
                for _ in g['data_paths']
            ]

            # 2. Merge env objects from every sim_config into the simulator.
            #    The first sim_config's objects are already present; add_env_objs
            #    skips duplicates by name.
            self._aggregate_env_objs_from_sim_configs()

            # 3. Build lookup structures for env swapping at runtime.
            self._env_by_name = {
                obj.name: obj for obj in self.simulator.all_env_planar_objs
            }
            self._group_env_names = [g['env_names'] for g in self._train_data_groups]

            # 4. Save canonical env object states now that all objects are registered.
            self._save_env_obj_originals()

            # 5. Replace the single merged DataLoader with per-group DataLoaders
            #    wrapped in a CombinedGroupDataLoader.
            train_group_dls = self._build_group_dataloaders(
                self.train_data_dict, self._train_data_groups, shuffle=True)
            val_group_dls = self._build_group_dataloaders(
                self.val_data_dict, self._val_data_groups, shuffle=False)

            self.train_dataloader = CombinedGroupDataLoader(train_group_dls, shuffle=True)
            self.val_dataloader   = CombinedGroupDataLoader(val_group_dls,   shuffle=False)

        else:
            # Legacy single-group path — behaviour is unchanged.
            super().__init__(training_config, criterion, dt, logger)
            self._env_by_name = None
            self._group_env_names = None
            self._save_env_obj_originals()

    # ------------------------------------------------------------------
    # Env object helpers
    # ------------------------------------------------------------------

    def _save_env_obj_originals(self):
        """Save original env object state so we can restore before each augmentation."""
        self._orig_env_states = []
        for obj in self.simulator.all_env_planar_objs:
            state = {
                'pos': obj.pos.clone(),
                'quat': obj.quat.clone(),
                'linear_vel': obj.linear_vel.clone(),
                'ang_vel': obj.ang_vel.clone(),
            }
            if isinstance(obj, StaticRectPlane):
                state['x_axis'] = obj.x_axis.clone()
                state['y_axis'] = obj.y_axis.clone()
                state['z_axis'] = obj.z_axis.clone()
            self._orig_env_states.append(state)

    def _restore_env_objs(self):
        """Restore env objects to their original (non-augmented) state."""
        for obj, orig in zip(self.simulator.all_env_planar_objs, self._orig_env_states):
            obj.pos = orig['pos'].clone()
            obj.quat = orig['quat'].clone()
            obj.linear_vel = orig['linear_vel'].clone()
            obj.ang_vel = orig['ang_vel'].clone()
            if isinstance(obj, StaticRectPlane):
                obj.x_axis = orig['x_axis'].clone()
                obj.y_axis = orig['y_axis'].clone()
                obj.z_axis = orig['z_axis'].clone()

    def _aggregate_env_objs_from_sim_configs(self):
        """Load every sim_config and merge its env objects into the simulator.

        The first sim_config's objects are already present (created in
        get_simulator); add_env_objs skips duplicates by name.
        """
        for path in self._sim_config_paths:
            with open(path, 'r') as f:
                cfg = json.load(f)
            if 'environment' in cfg:
                self.simulator.add_env_objs(cfg['environment'])

    def _build_group_dataloaders(self, merged_data_dict, groups, shuffle):
        """Split merged_data_dict by trajectory count per group and create DataLoaders."""
        dataloaders = []
        offset = 0
        for group_id, group in enumerate(groups):
            n = len(group['data_paths'])
            group_data_dict = {
                k: v[offset:offset + n]
                for k, v in merged_data_dict.items()
                if isinstance(v, list)
            }
            dataset = PlanarEnvGroupDataset(
                group_data_dict,
                num_steps_fwd=self.num_steps_fwd,
                dt=self.dt,
                num_ctrls_hist=self.num_ctrls_hist,
                env_group_id=group_id,
            )
            dataloaders.append(DataLoader(
                dataset,
                batch_size=self.batch_size_per_step,
                shuffle=shuffle,
                collate_fn=dataset.collate_fn,
            ))
            offset += n
        return dataloaders

    # ------------------------------------------------------------------
    # Simulator construction
    # ------------------------------------------------------------------

    def get_simulator(self):
        if self.load_sim and self.load_sim_path:
            sim = torch.load(self.load_sim_path, map_location="cpu", weights_only=False)
            sim.reset()
            # sim.run_compile()
            sim.cpu()
            print("Loaded simulator")
        else:
            sim_config_cpy = deepcopy(self.sim_config)
            sim_config_cpy.pop('gravity')
            sim_config_cpy.pop('contact_params')

            sim = PlanarEnvTensegrityGNNSimulator(
                **sim_config_cpy,
                num_sims=self.num_sims,
                num_ctrls_hist=self.num_ctrls_hist,
                torch_compile=False,
            )

        return sim

    # ------------------------------------------------------------------
    # Device movement
    # ------------------------------------------------------------------

    def to(self, device):
        """Move trainer to device, including saved env object states."""
        super().to(device)
        for orig in self._orig_env_states:
            for k, v in orig.items():
                if isinstance(v, torch.Tensor):
                    orig[k] = v.to(device)
        return self

    # ------------------------------------------------------------------
    # Data augmentation
    # ------------------------------------------------------------------

    def rotate_data_aug(self, batch_x, gt_end_pts):
        n = len(self.simulator.robot.rods)
        B = batch_x.shape[0]

        # Random angle about z-axis
        angle = 2 * torch.pi * (torch.rand((B, 1, 1), device=batch_x.device) - 0.5)

        # Build z-axis rotation quaternion: (B, 4, 1)
        w = torch.cos(angle / 2)
        xyz = torch.tensor([0, 0, 1],
                           dtype=self.dtype,
                           device=batch_x.device
                           ).reshape(1, 3, 1)
        xyz = xyz.repeat(B, 1, 1) * torch.sin(angle / 2)
        q_single = torch.hstack([w, xyz])  # (B, 4, 1)

        # For robot state: repeat q for n rods
        q = q_single.repeat(1, n, 1).reshape(-1, 4, 1)

        # Rotate robot state
        batch_x_rots = []
        for i in range(batch_x.shape[2]):
            batch_x_i = batch_x[..., i: i + 1].reshape(-1, 13, 1)
            pos = torch_quaternion.rotate_vec_quat(q, batch_x_i[:, :3])
            quat = torch_quaternion.quat_prod(q, batch_x_i[:, 3:7])
            linvel = torch_quaternion.rotate_vec_quat(q, batch_x_i[:, 7:10])
            angvel = torch_quaternion.rotate_vec_quat(q, batch_x_i[:, 10:])

            batch_x_rots.append(torch.hstack([
                pos, quat, linvel, angvel
            ]).reshape(-1, 13 * n, 1))

        batch_x_rots = torch.concat(batch_x_rots, dim=2)

        # Rotate gt endpoints
        gt_end_pts_rots = []
        for i in range(gt_end_pts.shape[2]):
            gt_end_pts_ = gt_end_pts[:, :, i: i + 1].reshape(-1, 6, 1)
            endpt_0_rot = torch_quaternion.rotate_vec_quat(q, gt_end_pts_[:, :3])
            endpt_1_rot = torch_quaternion.rotate_vec_quat(q, gt_end_pts_[:, 3:])
            gt_end_pts_rot = torch.hstack([
                endpt_0_rot, endpt_1_rot
            ]).reshape(-1, 6 * n, 1)

            gt_end_pts_rots.append(gt_end_pts_rot)

        gt_end_pts_rots = torch.concat(gt_end_pts_rots, dim=2)

        # Only rotate the currently-active env objects (not all of them).
        for obj in self.simulator.curr_env_planar_objs:
            rot_pos = torch_quaternion.rotate_vec_quat(q_single, obj.pos)
            rot_quat = torch_quaternion.quat_prod(q_single, obj.quat)
            obj.update_state(rot_pos, obj.linear_vel, rot_quat, obj.ang_vel)

        return batch_x_rots, gt_end_pts_rots

    # ------------------------------------------------------------------
    # Training loop
    # ------------------------------------------------------------------

    def run_one_epoch(self,
                      batches,
                      grad_required=True,
                      shuffle_data=False,
                      rot_aug=False) -> List[float]:
        if grad_required:
            self.simulator.train()
        else:
            self.simulator.eval()

        if shuffle_data:
            random.shuffle(batches)

        total_loss, total_other_losses = 0.0, []
        num_train, curr_batch = 0, 0
        for batch in tqdm.tqdm(batches):
            curr_batch += 1

            batch = {k: v.to(self.device) for k, v in batch.items()}
            num_train += batch['x'].shape[0]

            # Swap active env objects for this batch's group.
            # Must happen before rotate_data_aug so that rotation is applied
            # only to the currently-active subset of env objects.
            if self._env_by_name is not None and 'env_group_id' in batch:
                env_group_id = batch['env_group_id'][0, 0].item()
                self.simulator.curr_env_planar_objs = [
                    self._env_by_name[name]
                    for name in self._group_env_names[env_group_id]
                ]

            if rot_aug:
                batch['x'], batch['y'] = self.rotate_data_aug(batch['x'], batch['y'])

            graphs = self.batch_sim_ctrls(batch)
            losses = self.compute_node_loss(graphs, batch['y'], self.dt)

            self._restore_env_objs()

            # cable_dls = batch['next_act_lens'] - batch['act_len']
            act_lens, next_act_lens = batch['act_len'], batch['next_act_lens']
            cable_dls = next_act_lens - torch.concat([act_lens, next_act_lens[..., :-1]], dim=2)
            norm_cable_loss, cable_loss = self.compute_cable_dl_loss(graphs, cable_dls)

            backward_loss = losses[0] + (5 / self.num_steps_fwd) * norm_cable_loss
            losses = [backward_loss, norm_cable_loss.detach().item(), cable_loss] + [l for l in losses[1:]]

            # If gradient updates required, run backward pass
            if grad_required:
                self.backward(backward_loss)

            total_loss += losses[0].detach().item() * batch['x'].shape[0]
            total_other_losses.append([
                l * batch['x'].shape[0] for l in losses[1:]
            ])

            if curr_batch % self.PRINT_STEP == 0:
                avg_other_losses = [
                    sum(l) / num_train
                    for l in zip(*total_other_losses)
                ]
                print(total_loss / num_train, avg_other_losses)

        total_loss /= num_train
        avg_other_losses = [
            sum(l) / num_train
            for l in zip(*total_other_losses)
        ]

        losses = [total_loss] + avg_other_losses

        return losses

    # ------------------------------------------------------------------
    # Evaluation
    # ------------------------------------------------------------------

    def evaluate_rollouts(self, data_dict):
        """Evaluate rollouts, swapping env objects per-group of trajectories.

        In single-group (legacy) mode, delegates directly to the parent.
        In multi-group mode, filters data_dict to each group's trajectories,
        sets the correct env objects, calls the parent evaluate_rollouts, and
        returns a weighted average of the per-group losses.
        """
        if self._env_by_name is None or 'env_group_ids' not in data_dict:
            return super().evaluate_rollouts(data_dict)

        total_n = len(data_dict['states'])
        accum_rollout = 0.0
        accum_n_step  = 0.0
        accum_other   = None

        for group_id, env_names in enumerate(self._group_env_names):
            group_idxs = [
                i for i, gid in enumerate(data_dict['env_group_ids'])
                if gid == group_id
            ]
            if not group_idxs:
                continue

            # Build a data_dict that contains only this group's trajectories.
            group_data_dict = {
                k: ([v[i] for i in group_idxs] if isinstance(v, list) else v)
                for k, v in data_dict.items()
            }

            # Activate this group's env objects.
            self.simulator.curr_env_planar_objs = [
                self._env_by_name[n] for n in env_names
            ]

            # super().evaluate_rollouts falls through to
            # TensegrityGNNTrainingEngine.evaluate_rollouts, which calls
            # self.eval_n_step_aheads and self.eval_rollout_fixed_ctrls via the
            # MRO — no infinite recursion since we do not override those two methods.
            rollout_loss, n_step_loss, other_losses = super().evaluate_rollouts(group_data_dict)

            n = len(group_idxs)
            accum_rollout += rollout_loss * n
            accum_n_step  += n_step_loss * n
            if accum_other is None:
                accum_other = [l * n for l in other_losses]
            else:
                accum_other = [a + b * n for a, b in zip(accum_other, other_losses)]

        self._restore_env_objs()

        return (
            accum_rollout / total_n,
            accum_n_step  / total_n,
            [l / total_n for l in (accum_other or [])],
        )
