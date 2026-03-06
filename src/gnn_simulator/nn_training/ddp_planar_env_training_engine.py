"""Distributed Data Parallel (DDP) subclass of PlanarEnvTensegrityMultiSimMultiStepMotorGNNTrainingEngine.

Usage (with torchrun):
    torchrun --nproc_per_node=N your_train_script.py

    # In your_train_script.py:
    import os
    import torch.distributed as dist

    dist.init_process_group("nccl")
    rank = int(os.environ["LOCAL_RANK"])
    world_size = int(os.environ["WORLD_SIZE"])

    trainer = DDPPlanarEnvTensegrityMultiSimMultiStepMotorGNNTrainingEngine(
        config, torch.nn.MSELoss(), dt, logger, rank=rank, world_size=world_size
    )
    trainer.run(num_epochs)

Notes:
- Caller is responsible for dist.init_process_group(...) before constructing this class.
- Caller is responsible for dist.destroy_process_group() after all training stages finish.
- Only the multi-group path ('train_data_groups' in config) is supported.
- eval_rollout_fixed_ctrls runs on CPU (DDP is temporarily unwrapped).
- Saved .pt files are stripped of the DDP wrapper so they load cleanly.
"""
import logging
from pathlib import Path
from typing import Dict, List, Union
import tqdm as _tqdm_module

import torch
import torch.distributed as dist
from torch.nn.modules.loss import _Loss
from torch.nn.parallel import DistributedDataParallel as DDP
from torch.utils.data import DataLoader, DistributedSampler

from gnn_simulator.nn_training.datasets.tensegrity_dataset import PlanarEnvGroupDataset
from gnn_simulator.nn_training.planar_env_tensegrity_gnn_training_engine import (
    CombinedGroupDataLoader,
    PlanarEnvTensegrityMultiSimMultiStepMotorGNNTrainingEngine,
)


class DDPPlanarEnvTensegrityMultiSimMultiStepMotorGNNTrainingEngine(
    PlanarEnvTensegrityMultiSimMultiStepMotorGNNTrainingEngine
):
    """Multi-GPU DDP variant. Inherits all behaviour; overrides only what DDP requires."""

    def __init__(
        self,
        training_config: Dict,
        criterion: _Loss,
        dt: Union[float, torch.Tensor],
        logger: logging.Logger,
        rank: int,
        world_size: int,
    ):
        # Set DDP state BEFORE super().__init__() so that the overridden
        # _build_group_dataloaders sees self.rank / self.world_size during construction.
        self.rank = rank
        self.world_size = world_size
        self._train_samplers: List[DistributedSampler] = []
        self._val_samplers: List[DistributedSampler] = []

        # Treat batch sizes in the config as *total* across all GPUs; divide
        # down to the per-GPU size before the parent builds DataLoaders.
        training_config = dict(training_config)  # shallow copy — don't mutate caller's dict
        for key in ("batch_size_per_step", "batch_size_per_update"):
            total = training_config.get(key, 0)
            assert total % world_size == 0, (
                f"{key}={total} is not divisible by world_size={world_size}"
            )
            training_config[key] = total // world_size

        super().__init__(training_config, criterion, dt, logger)

        # Move the whole engine to this rank's GPU.
        self.to(f"cuda:{rank}")

        # Wrap only the inner GNN with DDP.  All simulator methods (reset, run,
        # curr_env_planar_objs, …) remain directly accessible on self.simulator.
        self.simulator._encode_process_decode = DDP(
            self.simulator._encode_process_decode,
            device_ids=[rank],
            find_unused_parameters=False,
        )
        if hasattr(self.simulator._encode_process_decode, "_set_static_graph"):
            self.simulator._encode_process_decode._set_static_graph()

    # ------------------------------------------------------------------
    # DataLoader construction — adds DistributedSampler per group
    # ------------------------------------------------------------------

    def _build_group_dataloaders(self, merged_data_dict, groups, shuffle):
        """Like the parent, but each DataLoader uses a DistributedSampler."""
        dataloaders = []
        samplers = []
        offset = 0
        for group_id, group in enumerate(groups):
            n = len(group["data_paths"])
            group_data_dict = {
                k: v[offset: offset + n]
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
            sampler = DistributedSampler(
                dataset,
                num_replicas=self.world_size,
                rank=self.rank,
                shuffle=shuffle,
            )
            samplers.append(sampler)
            dataloaders.append(
                DataLoader(
                    dataset,
                    batch_size=self.batch_size_per_step,
                    sampler=sampler,
                    collate_fn=dataset.collate_fn,
                )
            )
            offset += n

        # Store samplers so train_epoch can call set_epoch() each epoch.
        if shuffle:
            self._train_samplers.extend(samplers)
        else:
            self._val_samplers.extend(samplers)

        return dataloaders

    # ------------------------------------------------------------------
    # Normalizer synchronisation after calibration
    # ------------------------------------------------------------------

    def compute_init_losses(self):
        # Only rank 0 runs the full normalizer calibration pass; other ranks wait.
        if self.rank == 0:
            losses = super().compute_init_losses()
        dist.barrier()

        # Broadcast normalizer accumulators from rank 0 so every rank computes
        # identical mean/std_w_eps.  (mean and std_w_eps are @property derived
        # from _acc_sum, _acc_sum_squared, and _acc_count — not stored tensors.)
        if not self.load_sim:
            for normalizer in self.simulator.data_processor.normalizers.values():
                if not hasattr(normalizer, "_acc_sum"):
                    continue
                dist.broadcast(normalizer._acc_sum, src=0)
                dist.broadcast(normalizer._acc_sum_squared, src=0)
                # _acc_count is int32; cast to float for broadcast, then restore.
                count_f = normalizer._acc_count.to(
                    dtype=normalizer._acc_sum.dtype,
                    device=normalizer._acc_sum.device,
                )
                dist.broadcast(count_f, src=0)
                normalizer._acc_count = count_f.to(normalizer._acc_count.dtype)

        # Non-rank-0 processes didn't run super(), so compute their init losses
        # now (normalizers are already calibrated, so this is just a forward pass).
        if self.rank != 0:
            losses = super().compute_init_losses()

        # Keep all ranks phase-aligned: no rank may start train_epoch while
        # others are still running compute_init_losses forward passes.
        dist.barrier()
        return losses

    # ------------------------------------------------------------------
    # Progress bar — suppress on non-rank-0 processes
    # ------------------------------------------------------------------

    def run_one_epoch(self, batches, grad_required=True, shuffle_data=False, rot_aug=False):
        if self.rank == 0:
            return super().run_one_epoch(batches, grad_required, shuffle_data, rot_aug)

        # Non-rank-0: swap tqdm.tqdm with a passthrough so the parent's
        # `for batch in tqdm.tqdm(batches)` loop is silent.  The patch is
        # scoped to this call; DDP uses separate processes so there are no
        # thread-safety concerns with module-level state.
        original_tqdm = _tqdm_module.tqdm
        _tqdm_module.tqdm = lambda iterable=None, *args, **kwargs: iterable
        try:
            return super().run_one_epoch(batches, grad_required, shuffle_data, rot_aug)
        finally:
            _tqdm_module.tqdm = original_tqdm

    # ------------------------------------------------------------------
    # Training epoch — sampler epoch + rank-0 checkpoint saving
    # ------------------------------------------------------------------

    def train_epoch(self):
        # Advance samplers so each epoch sees a different data ordering.
        for s in self._train_samplers:
            s.set_epoch(self.epoch_num)
        train_losses = self.run_one_epoch(self.train_dataloader, rot_aug=True)

        if train_losses[0] < self.best_train_loss:
            self.best_train_loss = train_losses[0]
            self.num_no_improve = 0
        else:
            self.num_no_improve += 1

        if self.num_no_improve > self.MAX_NO_IMPROVE:
            self.logger.info("No improvement, lowering learning rate")
            self.best_train_loss = train_losses[0]
            self.num_no_improve = 0
            for p in self.optimizer.param_groups:
                p["lr"] /= 2.0

        with torch.no_grad():
            if self.epoch_num % self.EVAL_STEPSIZE == 0:
                for s in self._val_samplers:
                    s.set_epoch(self.epoch_num)

                val_losses = self.run_one_epoch(self.val_dataloader, grad_required=False)

                self.simulator.eval()
                val_rollout_loss, val_n_steps_loss, val_other_rollout_losses = (
                    self.evaluate_rollouts(self.val_data_dict)
                )
                self.simulator.train()
            else:
                val_losses = [-9.0]
                val_rollout_loss = -9.0
                val_n_steps_loss = -9.0
                val_other_rollout_losses = []

        # Only rank 0 writes checkpoints; save with the DDP wrapper stripped.
        if self.rank == 0:
            if -9.0 < val_losses[0] < self.best_val_loss:
                self.best_val_loss = val_losses[0]
                self._save_simulator("best_loss_model.pt")
            if -9.0 < val_rollout_loss < self.best_rollout_loss:
                self.best_rollout_loss = val_rollout_loss
                self._save_simulator("best_rollout_model.pt")
            if -9.0 < val_n_steps_loss < self.best_n_step_rollout_loss:
                self.best_n_step_rollout_loss = val_n_steps_loss
                self._save_simulator("best_n_step_rollout_model.pt")

        return (
            train_losses,
            val_losses,
            [val_rollout_loss, val_n_steps_loss, *val_other_rollout_losses],
        )

    def _save_simulator(self, filename: str) -> None:
        """Save simulator with DDP wrapper stripped so the file loads without DDP."""
        epd = self.simulator._encode_process_decode
        self.simulator._encode_process_decode = epd.module  # unwrap DDP
        torch.save(self.simulator, Path(self.output_dir, filename))
        self.simulator._encode_process_decode = epd  # restore DDP wrapper

    # ------------------------------------------------------------------
    # Logging — rank 0 only
    # ------------------------------------------------------------------

    def log_status(self, losses) -> None:
        if self.rank == 0:
            super().log_status(losses)

    # ------------------------------------------------------------------
    # Rollout evaluation — unwrap DDP, run on CPU, then rewrap
    # ------------------------------------------------------------------

    def eval_rollout_fixed_ctrls(self, data_dict):
        # Strip DDP wrapper so the simulator can be freely moved to CPU.
        ddp_module = self.simulator._encode_process_decode
        self.simulator._encode_process_decode = ddp_module.module

        # The base implementation moves everything to CPU, runs the rollout,
        # then moves back to self.device.  With the GNN unwrapped this works.
        try:
            result = super().eval_rollout_fixed_ctrls(data_dict)
        finally:
            # Restore DDP wrapper (even if the rollout raised).
            self.simulator._encode_process_decode = ddp_module

        return result

    # ------------------------------------------------------------------
    # Backward — suppress DDP all-reduce on intermediate accumulation steps
    # ------------------------------------------------------------------

    def backward(self, loss: torch.Tensor) -> None:
        """Gradient accumulation with DDP.

        DDP normally all-reduces after every .backward().  With gradient
        accumulation we only want the all-reduce to fire on the *last*
        accumulation step (just before optimizer.step()), so we use
        no_sync() to suppress communication on the intermediate steps.
        This ensures that when optimizer.step() is called, all GPUs have
        fully aggregated gradients.
        """
        ddp_gnn = self.simulator._encode_process_decode
        is_last_accum = self.curr_accum_step >= self.num_grad_accum

        if is_last_accum:
            # Final accumulation step: allow DDP all-reduce to fire.
            (loss / self.num_grad_accum).backward()
        else:
            # Intermediate step: accumulate locally without cross-GPU sync.
            with ddp_gnn.no_sync():
                (loss / self.num_grad_accum).backward()

        if is_last_accum:
            self.optimizer.step()
            self.optimizer.zero_grad()
            self.curr_accum_step = 1
        else:
            self.curr_accum_step += 1

    # ------------------------------------------------------------------
    # Run — barrier + process-group cleanup
    # ------------------------------------------------------------------

    def run(self, num_epochs: int):
        dist.barrier()
        super().run(num_epochs)
        dist.barrier()
