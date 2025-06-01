from functools import reduce
from typing import List, Union
from data_processing.direct_kinematics import build_analytical_jacobian, get_progressive_transformation_matrixes, build_jacobian, get_components_from_transf_matrix
import numpy as np
from internal_types.array import Array
from internal_types.urdf_model import Joint
from util.ros_publisher import RosPub
from data_loader.load_urdf import load_urdf
import math
import time


def inverse_kinematic(
    joints: List[Joint],
    desired_position: List[Union[float, None]],
    desired_orientation: List[Union[float, None]],
    initial_guess: Union[List[float], None] = None,
    beta_constant: float = 0.2,
    lambda_constant: float = 0.01,
    gamma_constant: float = 0.5,
    epsilon_constant: float = 1e-5,
    max_steps = 100,
    use_postural_task = False,
    desired_pose: Union[List[float], None] = None,
    desired_pose_relative_weights: Union[List[float], None] = None,
    weight_postural_task: float = 1e-4,
    visualize = False
) -> List[float]:
    """
    calculate the inverse kinematics of the robot, and return a list of numbers representing the Qs

    desired_position: list of X,Y,Z of the desired position
    desired_orientation: list of ro, phi, theta (euler angles) of the desired position.

    both of them can be None if no desired value exist
    """
    assert len(desired_position) == 3, "Position should have len 3 (x,y,z)"
    assert len(desired_orientation) == 3, "Orientation should have len 3 (ro, phi, and theta)"

    num_joints = len([x for x in joints if x.type != "fixed"])

    if initial_guess == None:
        initial_guess = [0.0] * num_joints
    else:
        assert len(initial_guess) == num_joints, f"The provided initial guess has len = {len(initial_guess)}, expect {num_joints} instead"

    if desired_pose is None:
        desired_pose = [0.0]*num_joints
    else:
        assert len(desired_pose) == num_joints, f"The desired pose has len = {len(desired_pose)}, expect {num_joints} instead"
    desired_pose = np.array(desired_pose, dtype = np.float64) #type: ignore

    if desired_pose_relative_weights is None:
        desired_pose_relative_weights = [1.0]*num_joints
    else:
        assert len(desired_pose_relative_weights) == num_joints, f"The desired pose relative_weights has len = {len(desired_pose_relative_weights)}, expect {num_joints} instead"
    desired_pose_relative_weights = np.array(desired_pose_relative_weights, dtype = np.float64) #type: ignore

    desired_pos_and_or = np.array([
        x if x is not None else np.nan
        for x in desired_position + desired_orientation
    ], dtype=np.float64)   

    current_guess = np.array(initial_guess, dtype=np.float64)

    def get_current_position(q: Array) -> Array:
        mat = get_progressive_transformation_matrixes(
            joints,
            list(q)
        )[-1]
        xyz, rpy = get_components_from_transf_matrix(mat)
        return np.array(xyz + rpy, dtype=np.float64)


    
    mm = np.matmul
    def mm_chain(*mats) -> Array:
        mat_array = [m for m in mats]
        return reduce(mm, mat_array)


    identity = np.identity(num_joints)

    # print(f"Desired position: {desired_pos_and_or}")
    for _ in range(max_steps):
        pos = get_current_position(current_guess)

        j = build_analytical_jacobian(joints, list(current_guess))
        # remove the components that are not part of our task space variables
        j = j[np.isnan(desired_pos_and_or) == False] #type: ignore

        error = pos - desired_pos_and_or #type: ignore
        # remove the NaN components
        error = error[np.isnan(error) == False]

        if use_postural_task:
            error = np.hstack((error, -weight_postural_task * desired_pose_relative_weights * (desired_pose - current_guess))) #type: ignore
            j_plus_identity = np.vstack((j, weight_postural_task * identity)) #type: ignore

            delta_q = -mm_chain(
                np.linalg.inv(mm(j.T,j) + weight_postural_task**2*identity), #type: ignore
                j_plus_identity.T,
                error
            )
        else:

            # print(np.linalg.inv(mm(j.T,j) + weight_postural_task**2*identity).shape)
            # print(j.T.shape)
            # print(error.shape)
            delta_q = -mm_chain(
                np.linalg.inv(mm(j.T,j) + lambda_constant * identity), #type: ignore
                j.T,
                error
            )

        error_abs = old_error = np.sum(error**2)**0.5
        # print(f"Current position: {pos}")
        # print(f"Error: {error} with abs={error_abs}")
        
        old_error = np.sum(error**2)**0.5

        alpha = 1.0
        while True:
            new_q = current_guess + alpha * delta_q


            error = get_current_position(new_q) - desired_pos_and_or #type: ignore

            # remove the NaN components
            error = error[np.isnan(error) == False]

            new_error: float = np.sum(error**2)**0.5

            if old_error - new_error >= gamma_constant * alpha * old_error:
                current_guess[:] = new_q[:]
                break
            else:
                alpha = alpha * beta_constant

        
        if new_error <= epsilon_constant:
            break


    print("final guess: ", current_guess)
    print("final error: ", new_error) #type: ignore

    if visualize:
        ros_pub = RosPub("arm", [j.name for j in joints if j.type != "fixed"])
        time.sleep(3)
        ros_pub.publish(current_guess) #type: ignore
        input("Press enter to quit")

    return list(current_guess)


def test_inverse_kinematics():
    joints = load_urdf()

    initial_guess: list[float] = [2, 1, 1, 0, 0]
    # position of the end effector (x,y,z)
    position: List[Union[float, None]] = [4,4,1]
    # orientation of the end effector (roll, pitch, yaw.) orient with 30 degrees angle ad the end like specifications
    orientation: List[Union[float, None]] = [None, None, None]
    # orientation: List[Union[float, None]] = [None, math.radians(-30), None]

    inverse_kinematic(joints, position, orientation, initial_guess, visualize=False)


def test_inverse_kinematics_with_postural():
    joints = load_urdf()

    initial_guess: list[float] = [2, 1, 1, 0, 0]
    # position of the end effector (x,y,z)
    position: List[Union[float, None]] = [1,2,1]
    # orientation of the end effector (roll, pitch, yaw.) orient with 30 degrees angle ad the end like specifications
    # orientation: List[Union[float, None]] = [None, None, None]
    orientation: List[Union[float, None]] = [None, math.radians(-30), None]

    # desired_pose: List[float] = [0,0,0,0,0]
    desired_pose: List[float] = [0,0,0,0,0]
    # I only care about the position of the second joint, all of the others can do what they prefer
    desired_pose_relative_weights: List[float] = [0,1,0,0,0]

    inverse_kinematic(
        joints,
        position,
        orientation,
        initial_guess,
        visualize=True,
        use_postural_task=True,
        desired_pose=desired_pose,
        weight_postural_task=0.6,
        desired_pose_relative_weights=desired_pose_relative_weights
    )
