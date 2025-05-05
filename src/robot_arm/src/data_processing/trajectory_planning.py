from typing import List, Union
from internal_types.array import Array
import numpy as np

class Trajectory:
    def __init__(self, initial_position: Union[Array, List[float]], final_position: Union[Array, List[float]], time: float):
        initial_position = [float(x) for x in initial_position]
        final_position = [float(x) for x in final_position]

        self.pow = np.array(range(6)).astype(np.float64)
        self.times = np.ones((6,), dtype = np.float64)
        self.coefficients: Array = np.stack([ #type: ignore
            get_trajectory_coefficients(start, end, time)     
            for start, end in zip(initial_position, final_position)
        ])


    def __call__(self, time: float) -> Array:
        self.times[:] = time
        return np.sum(
            self.coefficients * self.times ** self.pow, #type: ignore
            axis=1
        )

            

def get_trajectory_coefficients(initial_q: float, final_q: float, total_time: float) -> Array:


    q = np.array([initial_q,0,0,final_q,0,0])

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


def test():

    t = Trajectory([1,2,3], [3,2,1], 4)

    for i in range(101):
        print(t(4*i/100))
