from time import sleep
from typing import Callable, TypeVar, List
from pinocchio import RobotWrapper
from constants.path import URDF_FILE_PATH
import pinocchio as pin
import numpy as np
import numpy as np
from data_loader.load_urdf import load_urdf
from internal_types.array import Array
from util.ros_publisher import RosPub


#wrapper around pinocchio, to add type annotations
def rnea(robot: RobotWrapper, q: Array, qd: Array, qdd: Array) -> Array:
    return pin.rnea(robot.model, robot.data, q, qd, qdd) #type: ignore

def run_dynamics():
    # Notes from professor at page 26
    # https://drive.google.com/drive/folders/17MCKvglqk94NsF3zj4OkmZN5_yU-wxtq


    joints = load_urdf()
    joints = [x for x in joints if x.type != "fixed"]


    VISUAL = True
    N_JOINS = len(joints)
    DT = 0.001 #simulation delta time
    SLEEP_TIME = 0.001 # sleep time to slow down visual


    T = TypeVar("T")
    def make_array(items: List[T], fn: Callable[[T], float], default: float):
        array = []
        for x in items:
            try:
                array.append(fn(x))
            except:
                array.append(default)
        return np.asarray(array)


    damping_coefficients = make_array(joints, lambda x: x.dynamics.damping, 0) #type: ignore
    friction_coefficients = make_array(joints, lambda x: x.dynamics.friction, 0) #type: ignore
    q_max = make_array(joints, lambda x: x.limit.upper, np.inf) #type: ignore
    q_min = make_array(joints, lambda x: x.limit.lower, -np.inf) #type: ignore
    efforts = make_array(joints, lambda x: x.limit.effort, 0) #type: ignore 

    if VISUAL:
        ros_pub = RosPub("arm", [j.name for j in joints])

    # sleep to wait rviz to start
    sleep(3)
    zero_vec = np.array([0.0] * N_JOINS)

    q =   np.array([0.0] * N_JOINS)
    qd =  np.array([0.0] * N_JOINS)
    qdd = np.array([0.0] * N_JOINS)

    ## initial velocity
    qd[1] = 1
    qd[2] = 2

    robot = RobotWrapper.BuildFromURDF(URDF_FILE_PATH)

    for i in range(10000):
        
        # non linear effects (due to inertia and gravity)
        non_linear_effects = -rnea(robot, q, qd, zero_vec)

        # q[2] = 0
        # qd[2] = 0

        # inertia matrix
        M = np.zeros((N_JOINS, N_JOINS))
        for i in range(N_JOINS):
            ei = zero_vec.copy()
            ei[i] = 1
            # the torque generated to keep G stable
            g_effect = rnea(robot, q, zero_vec, zero_vec)

            # the torques required to generate an unit acceleration on vector i 
            tau = rnea(robot, q, zero_vec,ei)
            
            # compensation for gravity effect
            tau -= g_effect #type: ignore

            # build the inertia matrix partially
            M[:,i] = tau

        friction = -np.minimum(np.abs(non_linear_effects), friction_coefficients) * np.sign(qd) #type: ignore
        damping = -damping_coefficients * qd
        end_stop =  (q > q_max) * (efforts * (q_max - q) ) +  (q  < q_min) * (efforts * (q_min - q) ) #type: ignore

        qdd = np.linalg.inv(M).dot(non_linear_effects + friction + damping + end_stop) #type: ignore

        # integration
        qd = qd + qdd * DT
        q = q + qd * DT + 0.5 * qdd * DT**2

        if VISUAL:
            ros_pub.publish(q, qd) #type: ignore

        sleep(SLEEP_TIME)


