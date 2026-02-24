from pathlib import Path
from mujoco_simulator.tensegrity_mjc_simulation import TensegrityMuJoCoSimulator

xml_path = Path("src/mujoco_simulator/xml_models/3prism_real_upscaled_obs_course.xml")
sim = TensegrityMuJoCoSimulator(xml_path, visualize=False, attach_type='real_attach')

# Check gravity setting
print(f"=== scratch.py diagnostics ===")
print(f"Gravity: {sim.mjc_model.opt.gravity}")
print(f"timestep: {sim.mjc_model.opt.timestep}")
print(f"dt: {sim.dt}")
print(f"Initial mjc_data.time: {sim.mjc_data.time}")
print()

# Lift the object 12m
qpos = sim.mjc_data.qpos.reshape(-1, 7)
qpos[:, 2] += 12.0
sim.mjc_data.qpos = qpos.flatten()

# Zero out velocities to start fresh
sim.mjc_data.qvel[:] = 0

for i in range(100):
    sim.sim_step(controls=None)
    vels = sim.get_vels()
    z_velocities = vels[:, 2]
    print(f"mjc_time: {sim.mjc_data.time:.4f}, z_vel: {z_velocities}")
