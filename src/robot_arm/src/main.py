#!/usr/bin/env python3

from constants.path import URDF_FILE_PATH
from data_loader.load_urdf import load_urdf
print("hello world from ros!", URDF_FILE_PATH)
load_urdf()
