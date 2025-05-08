#!/usr/bin/env python3

from constants.path import URDF_FILE_PATH
from data_loader.load_urdf import load_urdf
from data_processing.dynamics import run_dynamics
from data_processing.trajectory_planning import visualize_trajectory

# print(load_urdf())
# exit(0)
visualize_trajectory()
