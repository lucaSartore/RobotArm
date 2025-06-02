from os import times_result
from time import sleep
from typing import Callable, TypeVar, List, Optional
from pinocchio import RobotWrapper
from constants.path import URDF_FILE_PATH
import pinocchio as pin
import numpy as np
import numpy as np
from data_loader.load_urdf import load_urdf
from internal_types.array import Array
from util.ros_publisher import RosPub
import typing

if typing.TYPE_CHECKING:
    from data_processing.controller import Controller

#wrapper around pinocchio, to add type annotations
def rnea(robot: RobotWrapper, q: Array, qd: Array, qdd: Array) -> Array:
    return pin.rnea(robot.model, robot.data, q, qd, qdd) #type: ignore


T = TypeVar("T")
class Simulator():
    
    def __init__(
        self,
        visual = True,
        dt = 0.001, #simulation delta time (seconds)
        total_time = 10, #total simulation time (seconds)
        sleep_time = 0.001, # sleep time to slow down visual (seconds)
        controller: Optional["Controller"] = None,
    ):
        joints = load_urdf()
        # constants used for simulation
        self.joints = [x for x in joints if x.type != "fixed"]
        self.n_joints = len(self.joints)
        self.visual = visual
        self.dt = dt
        self.total_time = total_time
        self.sleep_time = sleep_time
        self.controller = controller
        self.zero_vec = np.array([0.0] * self.n_joints)
        self.robot = RobotWrapper.BuildFromURDF(URDF_FILE_PATH)

        self.damping_coefficients = \
            Simulator.make_array(self.joints, lambda x: x.dynamics.damping, 0) #type: ignore
        self.friction_coefficients = \
            Simulator.make_array(self.joints, lambda x: x.dynamics.friction, 0) #type: ignore
        self.q_max = \
            Simulator.make_array(self.joints, lambda x: x.limit.upper, np.inf) #type: ignore
        self.q_min = \
            Simulator.make_array(self.joints, lambda x: x.limit.lower, -np.inf) #type: ignore
        self.efforts = \
            Simulator.make_array(self.joints, lambda x: x.limit.effort, 0) #type: ignore 

        # simulation variables
        self.q =   np.array([0.0] * self.n_joints)
        self.qd =  np.array([0.0] * self.n_joints)
        self.qdd = np.array([0.0] * self.n_joints)
        self.time = 0.0

        
        # simulation variables that are cashed
        self._M: Optional[Array] = None
        self._non_linear_effects: Optional[Array] = None

    def time_step(self):


        friction = -np.minimum(np.abs(self.non_linear_effects), self.friction_coefficients) * np.sign(self.qd) #type: ignore
        damping = -self.damping_coefficients * self.qd
        end_stop = (self.q > self.q_max) * (self.efforts * (self.q_max - self.q) ) #type: ignore
        end_stop += (self.q  < self.q_min) * (self.efforts * (self.q_min - self.q) ) #type: ignore

        force = self.non_linear_effects + friction + damping + end_stop

        if self.controller is not None:
            force += self.controller.get_torque(self) #type: ignore

        self.qdd = np.linalg.inv(self.M).dot(force) #type: ignore

        # integration
        self.qd = self.qd + self.qdd * self.dt
        self.q = self.q + self.qd * self.dt + 0.5 * self.qdd * self.dt **2

        
        #reset the property that will need to be  re-calculated as the robot has moved:
        self._M = None
        self._non_linear_effects = None


    def run(self):

        if self.visual:
            ros_pub = RosPub("arm", [j.name for j in self.joints])
            sleep(3)
        
        while self.time < self.total_time:
            self.time_step()
            self.time += self.dt

            if self.visual:
                ros_pub.publish(self.q, self.qd) #type: ignore

            sleep(self.sleep_time)

    @property
    def M(self) -> Array:
        if self._M is None:
            self._M = np.zeros((self.n_joints, self.n_joints))
            for i in range(self.n_joints):
                ei = self.zero_vec.copy()
                ei[i] = 1
                # the torque generated to keep G stable
                g_effect = rnea(self.robot, self.q, self.zero_vec, self.zero_vec)

                # the torques required to generate an unit acceleration on vector i 
                tau = rnea(self.robot, self.q, self.zero_vec,ei)
                
                # compensation for gravity effect
                tau -= g_effect #type: ignore

                # build the inertia matrix partially
                self._M[:,i] = tau

        return self._M
         
    @property
    def non_linear_effects(self) -> Array:
        if self._non_linear_effects is None:
            self._non_linear_effects = -rnea(self.robot, self.q, self.qd, self.zero_vec)
        assert self._non_linear_effects is not None
        return self._non_linear_effects

    @staticmethod
    def make_array(items: List[T], fn: Callable[[T], float], default: float):
        array = []
        for x in items:
            try:
                array.append(fn(x))
            except:
                array.append(default)
        return np.asarray(array)


def test_dynamics():
    simulator = Simulator()
    simulator.run()


def test_dynamics_with_initial_velocity():
    simulator = Simulator()
    simulator.q[2] = 3
    simulator.qd[0] = 1
    simulator.run()
