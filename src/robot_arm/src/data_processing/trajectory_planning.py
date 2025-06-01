import time
from typing import List, Union
from data_loader.load_urdf import load_urdf
from internal_types.array import Array
import numpy as np

from util.ros_publisher import RosPub

class Trajectory:
    def __init__(self, initial_position: Union[Array, List[float]], final_position: Union[Array, List[float]], time: float):
        initial_position = [float(x) for x in initial_position]
        final_position = [float(x) for x in final_position]

        self.time = time
        self.pow = np.array(range(6)).astype(np.float64)
        self.times = np.ones((6,), dtype = np.float64)
        self.coefficients: Array = np.stack([ #type: ignore
            get_trajectory_coefficients(start, end, time)     
            for start, end in zip(initial_position, final_position)
        ])


    def __call__(self, time: float) -> Array:
        return self.get(time)

    def get(self, time: float) -> Array:
        time = np.clip(time, 0, self.time) #type: ignore
        self.times[:] = time
        return np.sum(
            self.coefficients * self.times ** self.pow, #type: ignore
            axis=1
        )

    def get_derivative(self, time: float) -> Array:
        # todo: this can be improved by mathematically differentiate the polynomial trajectory
        return (
            self.get(time + 0.001) - self.get(time) #type: ignore
        ) / 0.001
            

def get_trajectory_coefficients(initial_position: float, final_position: float, total_time: float) -> Array:


    q = np.array([initial_position,0,0,final_position,0,0])

    t = total_time
    
    mat = [
        [1, 0, 0,    0,     0,       0       ],
        [0, 1, 0,    0,     0,       0       ],
        [0, 0, 2,    0,     0,       0       ],
        [1, t, t**2, t**3,  t**4,    t**5    ],
        [0, 1, 2*t,  3*t**2,4*t**3,  5*t**4  ],
        [0, 0, 2,    6*t,   12*t**2, 20*t**3 ]
    ]

    mat = np.linalg.inv(mat) #type: ignore

    return np.matmul(mat, q) #type: ignore


def visualize_trajectory():

    TIME_STEP = 0.1
    DELAY = 0.1
    TOTAL_TIME = 10
    start_position: List[float] = [0,0,0,0,0]
    end_position: List[float] = [3.14,1,2,-3.14/2,1]

    trajectory = Trajectory(start_position, end_position, TOTAL_TIME)

    joints = load_urdf()
    joints = [x for x in joints if x.type != "fixed"]
    ros_pub = RosPub("arm", [j.name for j in joints])
    time.sleep(3) #wait for rviz to start

    t: float = 0

    while t < TOTAL_TIME:

        position = trajectory(t)

        ros_pub.publish(position)
        
        t += TIME_STEP
        time.sleep(DELAY)

    #error at the end should be lower then threshold
    assert ((position - np.array(end_position))**2).mean() <= 0.001 #type: ignore
