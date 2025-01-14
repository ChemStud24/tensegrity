import os
import json
from argparse import ArgumentParser

import numpy as np
from matplotlib import pyplot as plt


if __name__ == '__main__':
    parser = ArgumentParser("pose evalutation")
    parser.add_argument("--dataset", default="dataset")
    parser.add_argument("-v", "--video", default="monday_roll15")
    args = parser.parse_args()

    num_frames = len(os.listdir(os.path.join(args.dataset, args.video, 'data')))
    # num_frames = 400

    timestamps = []
    for i in range(num_frames):
        info_path = os.path.join(args.dataset, args.video, 'data', '%04d.json'%i)
        with open(info_path, 'r') as f:
            info = json.load(f)

        timestamp = info['header']['secs']
        timestamps.append(timestamp)

    timestamps = np.array(timestamps)

    time_intervals = timestamps[1:] - timestamps[:-1]
    mean = time_intervals.mean()
    std = np.sqrt(np.sum((time_intervals - mean)**2) / (num_frames - 1))
    plt.plot(range(num_frames - 1), time_intervals)
    plt.xlabel('frame id')
    plt.ylabel('time interval (s)')
    plt.title('video: %s\nmean = %.3f, std = %.3fs' %(args.video, mean, std))
    plt.grid(True)
    plt.show()
