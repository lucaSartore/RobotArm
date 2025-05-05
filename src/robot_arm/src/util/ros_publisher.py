from typing import Union, List
import numpy as np
import rospy as ros
from sensor_msgs.msg import JointState
import numpy as np
import roslaunch
import rospkg
from internal_types.array import Array

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
