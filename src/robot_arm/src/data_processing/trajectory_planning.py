from abc import abstractmethod
import time
from typing import List, Union
from data_loader.load_urdf import load_urdf
from internal_types.array import Array
import numpy as np

from util.ros_publisher import RosPub


class Trajectory:

    def __call__(self, time: float) -> Array:
        return self.get(time)

    @abstractmethod
    def get(self, time: float) -> Array:
        ...

    def get_derivative(self, time: float) -> Array:
        return (
            self.get(time + 0.001) - self.get(time)  # type: ignore
        ) / 0.001



class JointTrajectory(Trajectory):
    def __init__(
        self,
        time_steps: List[float],
        position: List[Union[Array, List[float]]],
        velocities: List[Union[Array, List[float]]],
    ):
        assert len(time_steps) == len(position) == len(velocities)
        assert time_steps[0] == 0

        self.time_steps = time_steps

        self.trajectories: List[SimpleTrajectory] = [
            SimpleTrajectory(
                initial_position=position[i],
                final_position=position[i + 1],
                time=time_steps[i + 1] - time_steps[i],
                initial_velocity=velocities[i],
                final_velocity=velocities[i + 1]
            )
            for i in range(len(time_steps) - 1)
        ]

    def get(self, time: float) -> Array:
        # Find the correct trajectory segment
        for i in range(len(self.time_steps) - 1):
            if self.time_steps[i] <= time < self.time_steps[i + 1]:
                local_time = time - self.time_steps[i]
                return self.trajectories[i](local_time)
        # If time is beyond the final step, return the last position
        return self.trajectories[-1](self.trajectories[-1].time)

    def get_derivative(self, time: float) -> Array:
        return (
            self.get(time + 0.001) - self.get(time)  # type: ignore
        ) / 0.001

    def __call__(self, time: float) -> Array:
        return self.get(time)


class SimpleTrajectory(Trajectory):
    def __init__(
        self,
        initial_position: Union[Array, List[float]],
        final_position: Union[Array, List[float]],
        time: float,
        initial_velocity: Union[Array, List[float], None] = None,
        final_velocity: Union[Array, List[float], None] = None
    ):
        initial_position = [float(x) for x in initial_position]
        final_position = [float(x) for x in final_position]

        initial_velocity = [0 for _ in final_position] if initial_velocity is None else initial_velocity
        final_velocity = [0 for _ in final_position] if final_velocity is None else final_velocity
        initial_velocity = [float(x) for x in initial_velocity]
        final_velocity = [float(x) for x in final_velocity]

        self.time = time
        self.pow = np.array(range(6)).astype(np.float64)
        self.times = np.ones((6,), dtype=np.float64)
        self.coefficients: Array = np.stack([  # type: ignore
            get_trajectory_coefficients(start, end, time, v0, vf)
            for start, end, v0, vf in zip(initial_position, final_position, initial_velocity, final_velocity)
        ])


    def get(self, time: float) -> Array:
        time = np.clip(time, 0, self.time)  # type: ignore
        self.times[:] = time
        return np.sum(
            self.coefficients * self.times ** self.pow,  # type: ignore
            axis=1
        )



def get_trajectory_coefficients(
        initial_position: float,
        final_position: float,
        total_time: float,
        initial_velocity: float = 0,
        final_velocity: float = 0
) -> Array:

    q = np.array([
        initial_position,
        initial_velocity,
        0,
        final_position,
        final_velocity,
        0
    ])

    t = total_time

    mat = [
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 2, 0, 0, 0],
        [1, t, t**2, t**3, t**4, t**5],
        [0, 1, 2*t, 3*t**2, 4*t**3, 5*t**4],
        [0, 0, 2, 6*t, 12*t**2, 20*t**3]
    ]

    mat = np.linalg.inv(mat)  # type: ignore

    return np.matmul(mat, q)  # type: ignore


def visualize_trajectory():
    TIME_STEP = 0.1
    DELAY = 0.1
    TOTAL_TIME = 10
    start_position: List[float] = [0, 0, 0, 0, 0]
    end_position: List[float] = [3.14, 1, 2, -3.14 / 2, 1]

    trajectory = SimpleTrajectory(start_position, end_position, TOTAL_TIME)

    joints = load_urdf()
    joints = [x for x in joints if x.type != "fixed"]
    ros_pub = RosPub("arm", [j.name for j in joints])
    time.sleep(3)  # wait for rviz to start

    t: float = 0

    while t < TOTAL_TIME:
        position = trajectory(t)
        ros_pub.publish(position)
        t += TIME_STEP
        time.sleep(DELAY)

    # error at the end should be lower than threshold
    assert ((position - np.array(end_position)) ** 2).mean() <= 0.001  # type: ignore
