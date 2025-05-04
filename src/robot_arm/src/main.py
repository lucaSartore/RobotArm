#!/usr/bin/env python3

from constants.path import URDF_FILE_PATH
from data_loader.load_urdf import load_urdf
from data_processing.dynamics import run_dynamics

# print(load_urdf())
# exit(0)
run_dynamics()
