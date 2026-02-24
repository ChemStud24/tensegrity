import numpy as np


class PID:
    def __init__(self,
                 k_p=6.0,
                 k_i=0.01,
                 k_d=0.5,
                 min_length=80,
                 RANGE=100,
                 tol=0.1,
                 sys_precision=np.float64,
                 rest_len_bounds=(0.41, 2.69)):
        # self.last_control = np.zeros(n_motor)
        self.sys_precision = sys_precision
        self.last_error = None
        self.cum_error = None
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.min_length = min_length / 100.
        self.RANGE = RANGE / 100.
        self.tol = tol
        self.LEFT_RANGE = None
        self.RIGHT_RANGE = None
        self.done = None
        self.rest_len_bounds = rest_len_bounds

    def update_control_target_length(self, current_length, target_length):
        if self.cum_error is None:
            self.cum_error = np.zeros(current_length.shape, dtype=current_length.dtype)

        u = np.zeros(current_length.shape, dtype=current_length.dtype)
        RANGE = 1.0
        diff = current_length - target_length
        error = diff / RANGE

        high_error = np.abs(error) >= 0.05
        d_error = np.zeros(current_length.shape, dtype=current_length.dtype) \
            if self.last_error is None else error - self.last_error
        self.cum_error += error
        self.last_error = error

        u[high_error] = (self.k_p * error[high_error]
                         + self.k_i * self.cum_error[high_error]
                         + self.k_d * d_error[high_error])
        u = np.clip(u, a_min=-1, a_max=1)

        return u

    def update_control_by_target_gait(self, current_length, target_gait, rest_length):
        if self.done is None:
            self.done = np.array([False])

        if self.cum_error is None:
            self.cum_error = np.zeros((1), dtype=current_length.dtype)

        u = np.zeros((1), dtype=current_length.dtype)

        min_length = self.min_length
        range_ = np.clip(self.RANGE, a_min=1e-5, a_max=999999)

        position = (current_length - min_length) / range_

        if self.done:
            return u, position

        target_length = min_length + range_ * target_gait
        error = np.array([position - target_gait], dtype=current_length.dtype)

        low_error_cond1 = np.abs(error) < self.tol
        low_error_cond2 = np.abs(current_length - target_length) < 0.1
        low_error_cond3 = np.logical_and(target_gait == 0, position < 0)

        low_error = np.logical_or(
            np.logical_or(self.done, low_error_cond1),
            np.logical_or(low_error_cond2, low_error_cond3)
        )

        self.done[low_error] = True

        d_error = np.zeros(error.shape, dtype=error.dtype) \
            if self.last_error is None else error - self.last_error
        self.cum_error += error
        self.last_error = error

        u[~low_error] = (self.k_p * error[~low_error]
                         + self.k_i * self.cum_error[~low_error]
                         + self.k_d * d_error[~low_error])

        u = np.clip(u, a_min=-1, a_max=1)

        if ((self.rest_len_bounds[1] < rest_length and u < 0.)
                or (rest_length < self.rest_len_bounds[0] and u > 0.)):
            u[:] = 0
            # self.done = np.array([True])

        slack = np.logical_and(current_length < rest_length, u < 0)
        u[slack] = 0

        return u, position

    def reset(self):
        self.last_error = None
        self.cum_error = None
        self.done = None

    def set_range(self, RANGE):
        self.LEFT_RANGE = RANGE[0] / 100.
        self.RIGHT_RANGE = RANGE[1] / 100.
        # self.RANGE = RANGE / 100.

    def set_min_length(self, min_length):
        self.min_length = min_length / 100.
