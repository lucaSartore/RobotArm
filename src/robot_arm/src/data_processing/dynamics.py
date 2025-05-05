from time import sleep
from typing import Callable, TypeVar, Union, List
from pinocchio import RobotWrapper
from constants.path import URDF_FILE_PATH
import pinocchio as pin
import numpy as np
import rospy as ros
from sensor_msgs.msg import JointState
import numpy as np
import roslaunch
import rospkg
from data_loader.load_urdf import load_urdf

from internal_types.array import Array


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

class RosPub():
    def __init__(self, robot_name: str, axis_names: List[str]):
        #launch rviz
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False) #type: ignore
        roslaunch.configure_logging(uuid)
        package = rospkg.RosPack().get_path('robot_arm') + '/launch/visualize.launch'
        cli_args = [package, f'robot_name:={robot_name}','test_joints:=false']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)] #type: ignore
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file) #type: ignore
        parent.start()
        ros.loginfo("RVIZ started")
        self.joint_pub = ros.Publisher("/joint_states", JointState, queue_size=1)
        ros.init_node('sub_pub_node_python', anonymous=False, log_level=ros.FATAL)
        self.axis_names = axis_names

    def publish(self, q: Array, qd: Union[Array,None] = None, tau: Union[Array, None] = None):
        """
        publish a message updating the 
        """
        qd = qd if qd is not None else np.zeros(q.shape)
        tau = tau if tau is not None else np.zeros(q.shape)
                                
        msg = JointState()
        msg.header.stamp = ros.Time.now() 
            
        msg.name = self.axis_names
        msg.position = q                
        msg.velocity = qd                
        msg.effort = tau              
        
        self.joint_pub.publish(msg)
