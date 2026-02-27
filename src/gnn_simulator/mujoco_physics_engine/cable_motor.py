import numpy as np


class MotorState:
    def __init__(self, sys_precision=np.float64):
        super().__init__()
        self.iA_t = np.zeros(1, dtype=sys_precision)  # current through the coil
        self.omega_t = np.zeros(1, dtype=sys_precision)  # angular velocity
        self.tau_Jm = np.zeros(1, dtype=sys_precision)
        self.tau_motor = np.zeros(1, dtype=sys_precision)
        self.tau_stall = np.zeros(1, dtype=sys_precision)
        self.tau_load = np.zeros(1, dtype=sys_precision)

    def reset(self):
        self.omega_t = np.zeros(1, dtype=np.float64)


class DCMotor:
    def __init__(self,
                 speed=np.array(0.7),
                 sys_precision=np.float64):
        super().__init__()
        self.max_omega = np.array(220 * 2 * np.pi / 60., dtype=sys_precision)
        self.speed = speed
        self.motor_state = MotorState()

    def mod_ctrl_min_max(self, winch_r, delta_t, rest_length, cable_length, min_rest_length, max_rest_length):
        pre_omega = self.motor_state.omega_t.copy()

        def compute_ctrl(target_rest_length):
            # a = rest_length / cable_length
            dl = rest_length - target_rest_length
            ctrl = 2 * dl / (winch_r * delta_t * self.speed * self.max_omega * a)
            ctrl -= pre_omega / (self.speed * self.max_omega)
            return ctrl

        min_ctrl = compute_ctrl(min_rest_length)
        max_ctrl = compute_ctrl(max_rest_length)

        return min_ctrl, max_ctrl

    def compute_cable_length_delta(self, control, winch_r, dt):
        prev_omega = self.motor_state.omega_t.copy()
        self.motor_state.omega_t = (self.speed * self.max_omega * control).reshape(-1)
        delta_l = 0.5 * (prev_omega + self.motor_state.omega_t) * winch_r * dt

        return delta_l

    def reset_omega_t(self):
        self.motor_state.reset()
