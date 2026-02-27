import logging
import shutil
from pathlib import Path

import torch
import numpy as np

DEFAULT_DTYPE = torch.float32


def get_num_precision(precision: str) -> torch.dtype:
    if precision.lower() == 'float':
        return torch.float
    elif precision.lower() == 'float16':
        return torch.float16
    elif precision.lower() == 'float32':
        return torch.float32
    elif precision.lower() == 'float64':
        return torch.float64
    else:
        print("Precision unknown, defaulting to float32")
        return torch.float32


def save_curr_code(code_dir, output_dir):
    code_dir = Path(code_dir)
    output_dir = Path(output_dir)

    output_dir.mkdir(exist_ok=True)
    for p in code_dir.iterdir():
        if p.suffix in [".py", ".json", ".xml"]:
            shutil.copy(p, output_dir / p.name)
        elif p.is_dir() and not p.name.startswith("."):
            save_curr_code(p, output_dir / p.name)


def compute_num_steps(time_gap, dt, tol=1e-6):
    num_steps = int(time_gap // dt)
    gap = time_gap - dt * num_steps
    num_steps += 0 if gap < tol else 1
    # num_steps += int(math.ceil(time_gap - dt * num_steps))

    return num_steps

def linear_interp_to_regular_pts(ts, xs, dt):
    shifted_ts = ts - ts[0]
    reg_ts = np.linspace(0, shifted_ts[-1], dt)

    interp_xs = np.interp(reg_ts, shifted_ts, xs)
    return interp_xs


def setup_logger(output_dir):
    # Configure the logger
    logging.basicConfig(
        level=logging.INFO,  # Set the logging level
        format='%(asctime)s - %(levelname)s - %(message)s'  # Log message format
    )

    logger = logging.getLogger(__name__)

    # Remove file handlers
    for handler in logger.handlers[:]:  # Iterate over a copy of the list
        if isinstance(handler, logging.FileHandler):
            logger.removeHandler(handler)
            handler.close()  # Close the file handler to release the file lock

    # Create handlers
    file_handler = logging.FileHandler(Path(output_dir, 'log.txt'))

    # Set levels for handlers (optional)
    file_handler.setLevel(logging.INFO)

    # Create a formatter
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')

    # Add formatter to handlers
    file_handler.setFormatter(formatter)

    # Add handlers to the logger
    logger.addHandler(file_handler)

    return logger