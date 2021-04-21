import enum
import params

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
    
    # TO DO:
    # Check params.pegboard to see if the tool the user requested is even on the pegboard. If it's not error out.
    # If it's there, pick it up from its location and hold in handoff location
    # Wait until user pulls the robot arm out of it's impedence bubble
    # Let go of tool
    # Update params.pegboard and return to home position

    pegboard_tool_idx = np.where(np.array(params.pegboard) == tool.value)[0]
    if len(pegboard_tool_idx) != 0:
        # If there is a tool on the pegboard equal to the tool value
        # Pick tool from the first available pegboard spot
        print("Picking up tool {} from pegboard postion {}".format(tool.name, pegboard_tool_idx[0]))
        run_pick(pegboard_tool_idx[0], tool) # start aruco tag place function
        
        params.pegboard[pegboard_tool_idx[0]] = 0 # pegboard spot now empty
        print("Tool {} successfully picked from pegboard!".format(tool.name))

    else:
        # there is no tool
        print("Tool requested not on pegboard.")


def run_pick(tool_index, tool_id):

    #TODO FENG + JONATHAN
    print('Starting Robot')
    fa = FrankaArm()    

    print('Opening Grippers')
    #Open Gripper
    fa.open_gripper()

    print('Resetting Robot')
    #Reset Pose
    fa.reset_pose() 
    #Reset Joints
    fa.reset_joints()
    
    #pull array of positions from place function
    #pull offsets
    #create a drop off location
    #move robot arm from pick point to drop off location
    #at drop off location activate impedence control



if __name__ == "__main__": #unit testing code goes here
    class Tool(enum.IntEnum):
        test = 1

    params.validate() # initialize pegboard
    print(params.pegboard)
    execute(Tool.test)
    print(params.pegboard)
