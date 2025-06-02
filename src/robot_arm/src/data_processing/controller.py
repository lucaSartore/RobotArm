from data_loader.load_urdf import load_urdf
from data_processing.direct_kinematics import (
    build_jacobian,
    get_components_from_transf_matrix,
    get_progressive_transformation_matrixes
)
from internal_types.array import Array
from data_processing.trajectory_planning import Trajectory
import numpy as np
import typing
from typing import List, Optional
import matplotlib.pyplot as plt

if typing.TYPE_CHECKING:
    from data_processing.dynamics import Simulator

class Controller:

    def __init__(
            self,
            trajectory: Trajectory,
            position_filter: List[bool],
            desired_pose: List[float]
            
    ):
        self.trajectory = trajectory
        self.joints = load_urdf()
        self.joints_filtered = [x for x in self.joints if x.type != "fixed"]
        # filter an array of position to only keep the task variables that we
        # want to control (in this case the position and the pitch)
        self.position_filter = position_filter
        self.kp = 100
        self.kd = 25
        self.last_update = -1
        self.J_prev: Optional[Array] = None

        self.desired_pose = desired_pose

        self.errors: List[float] = []
        self.errors_dot: List[float] = []



    # time derivative of jacobian
    def get_Jd(self, J_current: Array, simulator: "Simulator") -> Array:
        prev = self.J_prev if self.J_prev is not None else J_current
        self.J_prev = J_current
        return (J_current-prev) / simulator.dt #type: ignore


    def get_torque(self, simulator: "Simulator") -> Array:

        # calculated using RENA algorithm
        M = simulator.M
        non_linear_effects = -simulator.non_linear_effects

        q = simulator.q
        qd = simulator.qd

        time = simulator.time

        # the desired position and position derivation
        p_desired = self.trajectory(time)
        pd_desired = self.trajectory.get_derivative(time)
        pdd_desired = self.trajectory.get_secon_derivative(time)


        Te = get_progressive_transformation_matrixes(self.joints, list(q))[-1]
        
        # size: 6x5
        J = build_jacobian(self.joints, list(q))
        Jd = self.get_Jd(J, simulator)

        # the actual position
        zyx, rpy = get_components_from_transf_matrix(Te)
        p = np.array(zyx + rpy)

        # the actual position derivation
        pd = np.matmul(J, qd) #type: ignore

        # filter out the task space variable that we don't want to control
        p = p[self.position_filter]
        pd = pd[self.position_filter]

        e =  p_desired - p
        ed = pd_desired - pd
        
        self.errors.append((e**2).sum()**0.5)
        self.errors_dot.append((ed**2).sum()**0.5)

        v = pdd_desired + self.kp * e + self.kd * ed
    
        J_pseudo_inv: Array = np.linalg.pinv(J) #type: ignore

        # remove some of the axis because I have some freedom of movement
        J_pseudo_inv_sliced = J_pseudo_inv[:,self.position_filter]
        Jd_sliced = Jd[self.position_filter, :]


        qdd_desired = np.matmul(
            J_pseudo_inv_sliced, #type: ignore
            v - np.matmul(Jd_sliced, qd) #type: ignore
        )

        # postural task
        postural_task: Array = 1.0 * (self.desired_pose - q)
        qdd_desired += np.matmul(
            (
                np.identity(len(self.joints_filtered)) -
                np.matmul(J_pseudo_inv, J) #type: ignore
            ),
            postural_task #type: ignore
        )

        u = np.matmul(M, qdd_desired) + non_linear_effects #type: ignore
        return u



    def plot_tracking_error(self):
        # Create a time axis assuming 1 unit per sample
        time = list(range(len(self.errors)))

        plt.figure(figsize=(12, 6))

        # Plot error
        plt.subplot(2, 1, 1)
        plt.plot(time, self.errors, label='Tracking Error')
        plt.title('Tracking Error')
        plt.xlabel('Time Step')
        plt.ylabel('Error')
        plt.grid(True)
        plt.legend()

        # Plot error derivative
        plt.subplot(2, 1, 2)
        plt.plot(time, self.errors_dot, label='Error Derivative (dot)', color='orange')
        plt.title('Tracking Error Derivative')
        plt.xlabel('Time Step')
        plt.ylabel('Error Derivative')
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        plt.show()

