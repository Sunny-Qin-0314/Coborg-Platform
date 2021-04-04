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
    #TO DO:
    #Check the table to see if the tool (aruco marker) is there. If not error out.
    #Check params.pegboard to see if there's space on the pegboard to place the tool. If no space error out.
    #Place the tool
    #update params.pegboard
    #return to home

if __name__ == "__main__": #unit testing code goes here
    tool = 1
    execute(tool)