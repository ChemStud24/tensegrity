import json

import torch
import numpy as np

from gnn_simulator.mujoco_physics_engine import ThreeBarTensegrityMuJoCoSimulator, \
    SixBarTensegrityMuJoCoSimulator
from gnn_simulator.robots.tensegrity import TensegrityRobot


def compare_mjc_and_gnn(robot_config,
                        mjc_xml_path,
                        tensegrity_type='3bar',
                        attach_type='center',
                        compare_motors=True,
                        compare_sites=False):
    np.set_printoptions(precision=64)
    assert tensegrity_type in ['3bar', '6bar']
    mjc_env_cls = ThreeBarTensegrityMuJoCoSimulator \
        if tensegrity_type == '3bar' else SixBarTensegrityMuJoCoSimulator
    mjc_env = mjc_env_cls(mjc_xml_path, visualize=False)
    mjc_env.forward()

    tensegrity = TensegrityRobot(robot_config)
    k=0

    # compare rods (mass, inertia, endcap position)
    rods = tensegrity.rods.values()
    sim_masses = torch.cat([r.mass for r in rods]).flatten().numpy()
    print(sim_masses.astype(np.float64))
    mjc_masses = mjc_env.mjc_model.body_mass[1:]
    diff = np.abs(sim_masses - mjc_masses)
    assert diff.max() < 1e-5, "Rod masses not the same"

    sim_inertia = torch.stack([torch.diag(r.I_body) for r in rods], dim=0).numpy()
    print(sim_inertia.astype(np.float64))
    mjc_inertia = mjc_env.mjc_model.body_inertia[1:]
    diff = np.abs(sim_inertia - mjc_inertia)
    assert diff.max() < 1e-4, "Rod inertia not the same"

    sim_end_pts = torch.vstack([e for r in rods for e in r.end_pts]).reshape(-1, 3).numpy()
    mjc_end_pts = np.stack([mjc_env.mjc_data.sensor(f'pos_s{i}').data for i in range(2 * mjc_env.num_rods)], axis=0)
    diff = np.abs(sim_end_pts - mjc_end_pts)
    assert diff.max() < 1e-6, "Rod end_pts not the same"

    # compare cables properties (stiffness, damping, rest lengths)
    cables = tensegrity.cables.values()

    sim_stiffness = torch.cat([c.stiffness for c in cables]).flatten().numpy()
    mjc_stiffness = mjc_env.mjc_model.tendon_stiffness.flatten()
    diff = np.abs(sim_stiffness - mjc_stiffness)
    assert diff.max() < 1e-6, "Cable stiffness not the same"

    sim_damping = torch.cat([c.damping for c in cables]).flatten().numpy()
    mjc_damping = mjc_env.mjc_model.tendon_damping.flatten()
    diff = np.abs(sim_damping - mjc_damping)
    assert diff.max() < 1e-6, "Cable damping not the same"

    sim_rest_lens = torch.cat([c.rest_length for c in cables]).flatten().numpy()
    mjc_rest_lens = mjc_env.mjc_model.tendon_lengthspring[:, :1].flatten()
    diff = np.abs(sim_rest_lens - mjc_rest_lens)
    assert diff.max() < 1e-6, "Cable rest lengths not the same"

    # compare motor params (speed, max_omega) if compare_motors flag is true
    sim_motors = [c.motor for c in cables if hasattr(c, 'motor')]
    mjc_motors = mjc_env.cable_motors

    sim_motor_speed = torch.cat([m.speed.reshape(1) for m in sim_motors]).flatten().numpy()
    mjc_motor_speed = np.concatenate([m.speed.reshape(1) for m in mjc_motors])
    diff = np.abs(sim_motor_speed - mjc_motor_speed)
    assert diff.max() < 1e-6, "Motor speed pct not the same"

    sim_max_omega = torch.cat([m.max_omega.reshape(1) for m in sim_motors]).flatten().numpy()
    mjc_max_omega = np.concatenate([m.max_omega.reshape(1) for m in mjc_motors])
    diff = np.abs(sim_max_omega - mjc_max_omega)
    assert diff.max() < 1e-6, "Motor max omega not the same"

    # compare cable attachment sites if compare_sites flag is true (naming must be the same)


if __name__ == '__main__':
    xml_path = '../mujoco_physics_engine/xml_models/3bar_new_platform_all_cables.xml'
    config_path = '../simulators/configs/new_3_bar_15_cables_5d_gnn_sim_config.json'
    with open(config_path, 'r') as f:
        config = json.load(f)['tensegrity_cfg']

    compare_mjc_and_gnn(config, xml_path, tensegrity_type='3bar')