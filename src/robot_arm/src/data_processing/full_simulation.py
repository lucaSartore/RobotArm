from numpy.typing import _128Bit
from data_loader.load_urdf import load_urdf
from data_processing.direct_kinematics import build_jacobian, get_components_from_transf_matrix, get_progressive_transformation_matrixes
from internal_types.array import Array
from data_processing.trajectory_planning import JointTrajectory, Trajectory
import numpy as np
import typing
from typing import Optional
import math
from data_processing.controller import Controller
from data_processing.dynamics import Simulator


def run_full_simulation():

    joints = load_urdf()
    Te = get_progressive_transformation_matrixes(joints, [0,0,0,0,0])[-1]
    xyz, rpy = get_components_from_transf_matrix(Te)
    initial_position = xyz + [rpy[1]]
    final_position = [1,2,1,math.radians(-30)]

    # trajectory = JointTrajectory(
    #     [0,4,7],
    #     [
    #         initial_position,
    #         [1,2,2,math.radians(-45)],
    #         final_position
    #     ],
    #     [
    #         [0,0,0,0],
    #         [0,0,0,0],
    #         [0,0,0,0]
    #     ]
    # )


    trajectory = JointTrajectory(
        [0,7],
        [
            initial_position,
            final_position
        ],
        [
            [0,0,0,0],
            [0,0,0,0]
        ]
    )

    controller = Controller(
        trajectory,
        [True,True,True,False,True,False]
    )
    
    simulator = Simulator(
        controller=controller,
        # visual=False,
        sleep_time=0
    )

    simulator.run()

    # controller.plot_tracking_error()

