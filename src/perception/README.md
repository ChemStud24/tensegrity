# tensegrity_perception
This is the official repository for the paper "6N-DoF Pose Tracking for Tensegrity Robot". [Paper Link](https://arxiv.org/abs/2205.14764)

This paper has been presented at the International Symposium on Robotics Research. [(ISRR 2022)](https://h2t-projects.webarchiv.kit.edu/ISRR2022/)

## Introduction
 This work aims to address the pose tracking problem of an N-bar tensegrity robots through a markerless, vision-based method, as well as novel, on-board sensors that can measure the length of the robot's cables.  In particular, an iterative optimization process is proposed to estimate the 6-DoF poses of each rigid element of a tensegrity robot from an RGB-D video as well as endcap distance measurements from the cable sensors. To ensure the pose estimates of rigid elements are physically feasible, i.e., they are not resulting in collisions between rods or with the environment, physical constraints are introduced during the optimization.

![](https://i.imgur.com/pzNl5ek.gif)

## Installation
1. Install Miniconda/Anaconda from [here](https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html)
2. `conda env create -f environment.yml`
3. `conda activate tensegrity`

## Demo Trajactories
Some demo trajactories could be downloaded [here](https://drive.google.com/drive/folders/1yOGHYOyp2WcwmNYEdUFf0qF-UtWxj8Hh?usp=sharing).

## Usage
1. Create a `dataset` folder in the repository and place the demo trajectories inside.
2. A bash script `tracking.sh` is provided, which contains commands to run the algorithm, evaluate, and generate videos for visualization.

## (Deprecated)
The original implementation (which requires CUDA and is much slower) accompanying the ISRR submission is in the `ISRR_version` branch.

## Citation
```
@misc{lu20226ndof,
    title={6N-DoF Pose Tracking for Tensegrity Robots},
    author={Shiyang Lu and William R. Johnson III and Kun Wang and Xiaonan Huang and Joran Booth and Rebecca Kramer-Bottiglio and Kostas Bekris},
    year={2022},
    eprint={2205.14764},
    archivePrefix={arXiv},
    primaryClass={cs.RO}
}
```