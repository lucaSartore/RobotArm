from internal_types.array import Array
from internal_types.urdf_model import Joint
from data_loader.load_urdf import load_urdf
from functools import reduce
import numpy as np

def build_transf_matrix(joint: Joint, q: float) -> Array:

    [yaw,pitch,roll] = joint.origin.rpy
    [x,y,z] = joint.origin.xyz

    cr = np.cos(roll)
    sr = np.sin(roll)

    cp = np.cos(pitch)
    sp = np.sin(pitch)
    
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    matrix = np.asarray(
        [
            [cr*cp, cr*sp*sy - sr*cy, cr*sp*cy + sr*sy, x],
            [sr*cp, sr*sp*sy + cr*cy, sr*sp*cy - cr*sy, y],
            [-sp,   cp*sy,            cp*cy,            z],
            [0,     0,                0,                1]
        ]
    )

    return matrix


def test():
    data = load_urdf()

    # data = data[0:2]
    matrixes = [
        build_transf_matrix(x,0) for x in data
    ]


    result = reduce(lambda a,b: np.matmul(a,b), matrixes)

    print(result)


