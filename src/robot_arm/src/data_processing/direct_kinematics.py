from typing import List
from internal_types.array import Array
from internal_types.urdf_model import Joint
from data_loader.load_urdf import load_urdf
import numpy as np

def build_transf_matrix_from_components(
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float
) -> Array:
    cr = np.cos(roll)
    sr = np.sin(roll)

    cp = np.cos(pitch)
    sp = np.sin(pitch)
    
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    return np.asarray(
        [
            [cr*cp, cr*sp*sy - sr*cy, cr*sp*cy + sr*sy, x],
            [sr*cp, sr*sp*sy + cr*cy, sr*sp*cy - cr*sy, y],
            [-sp,   cp*sy,            cp*cy,            z],
            [0,     0,                0,                1]
        ]
    )


def build_transf_matrix_from_join(joint: Joint, q: float) -> Array:
    """
    Calculate the homogeneous transformation matrix from of one joint given his q
    """
    # around x, around y, around z (fixed axis)
    [yaw, pitch, roll] = joint.origin.rpy
    [x,y,z] = joint.origin.xyz


    matrix = build_transf_matrix_from_components(x,y,z,roll,pitch,yaw)

    if joint.type == "prismatic":
        second_matrix = build_transf_matrix_from_components(0,0,q,0,0,0)
    elif joint.type == "revolute":
        second_matrix = build_transf_matrix_from_components(0,0,0,q,0,0)
    else:
        second_matrix = build_transf_matrix_from_components(0,0,0,0,0,0)

    return np.matmul(matrix, second_matrix) #type: ignore

def get_progressive_transformation_matrixes(joints: List[Joint], qs: List[float]) -> List[Array]:
    """
    Return a list of transformations matrix.
    given n = len(qs):
    the first n matrix transform from world frame to join i

    the last matrix transform from world frame to end effector

    the list has thus len = n+1
    """


    l1 = len([x for x in joints if x.type != "fixed"]) 
    l2 = len(qs)
    assert l1 == l2, f"The number of Qs passed should be the same as the number of not fixed joins, got {l1} and {l2} instead"

    current_matrix: Array = build_transf_matrix_from_components(0,0,0,0,0,0)

    to_return: List[Array] = []
    
    i = 0
    for joint in joints:

        if joint.type != "fixed":
            q = qs[i]
            i+=1
        else:
            q = 0.0

        joint_matrix = build_transf_matrix_from_join(joint, q)

        current_matrix = np.matmul(current_matrix, joint_matrix) #type: ignore

        if joint.type != "fixed":
            to_return.append(current_matrix.copy())

    to_return.append(current_matrix) # end effector matrix

    return to_return


def build_jacobian(joints: List[Joint], qs: List[float]) -> Array:

    jacobian = np.zeros(shape = (6, len(qs)), dtype = np.float64)

    transformation_matrixes = get_progressive_transformation_matrixes(joints, qs)

    t_ee = transformation_matrixes.pop()


    # slice the z component from the transformation matrix
    def z(mat: Array) -> Array:
        print(f"z of: \n{mat} is: {mat[0:3,2]}")
        return mat[0:3,2]

    # slice the p component from the transformation matrix
    def p(mat: Array) -> Array:
        return mat[0:3,3]

    # create teh column of the jacobian from a joint and his homogeneous transformation matrix
    def column(joint: Joint, mat: Array) -> Array:
        to_return = np.zeros(shape=(6,), dtype=np.float64)
        if joint.type == "prismatic":
            to_return[0:3] = z(mat)
        elif joint.type == "revolute":
            to_return[0:3] = np.cross(z(mat), (p(t_ee) - p(mat))) #type: ignore
            to_return[3:6] = z(mat)
        else:
            raise Exception("This code should be unreachable")
        return to_return

    joints = [x for x in joints if x.type != "fixed"]

    for i,(j,t) in enumerate(zip(joints, transformation_matrixes)):
        jacobian[:,i] = column(j,t)

    return jacobian


def test():
    data = load_urdf()
    # data = data[0:4]

    q = [
        0.0,
        0.0,
        0.0,
        0.0,
        -3.1415926/2,
    ]
    qd = np.asarray([
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ])


    print("\n ====================================== \n")
    print("HTM")
    [print(x, "\n") for x in get_progressive_transformation_matrixes(data, q)]

    j = build_jacobian(data, q)

    print("\n ====================================== \n")
    print("Jacobian")
    print(j)
    velocity = np.matmul(j,qd) #type: ignore
    print("\n ====================================== \n")
    print("Velocity")
    print(velocity)

