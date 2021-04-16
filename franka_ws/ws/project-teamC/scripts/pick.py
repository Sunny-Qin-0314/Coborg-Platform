import enum

from frankapy import FrankaArm
import rospy
import pickle as pkl
import numpy as np

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage, CartesianImpedanceSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

from frankapy.utils import min_jerk, min_jerk_weight

def execute(tool): 
    print(tool.value)
    pegboard_empspots_idx = np.where(np.array(params.pegboard) == tool.value)[0]

    #TO DO:
    #Check params.pegboard to see if the tool the user requested is even on the pegboard. If it's not error out.
    #If it's there, pick it up from its location and hold in handoff location
    #Wait until user pulls the robot arm out of it's impedence bubble
    #Let go of tool
    #Update params.pegboard and return to home position

if __name__ == "__main__": #unit testing code goes here
    tool = 1
    execute(tool)