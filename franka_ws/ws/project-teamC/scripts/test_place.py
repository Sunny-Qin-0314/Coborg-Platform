from frankapy import FrankaArm
import numpy as np
import argparse
import cv2
from cv_bridge import CvBridge
from autolab_core import RigidTransform, Point
from perception import CameraIntrinsics
from utils import *
from geometry_msgs.msg import Pose
import math

AZURE_KINECT_INTRINSICS = 'calib/azure_kinect.intr'
AZURE_KINECT_EXTRINSICS = 'calib/azure_kinect_overhead/azure_kinect_overhead_to_world.tf'

 
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

def quaternion_rotation_matrix(x,y,z,w):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = w
    q1 = x
    q2 = y
    q3 = z
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

def run_pick_up(available_pegboard_spot, tool_id):
    '''
    Pick up one tool and place it into the specific pegboard spot

    Inputs:
        available_pegboard_spot: int, the index of the availble position on the board [0,1,2,3]
        tool_id: int, the tool want to pick up. This will be used to read the correct marker pose topic [1,2,3,4]
                tool1: aruco marker 100
                tool2: aruco marker 153
                tool3: aruco marker 80
                tool4: aruco marker 33
    '''
    parser = argparse.ArgumentParser()
    parser.add_argument('--intrinsics_file_path', type=str, default=AZURE_KINECT_INTRINSICS)
    parser.add_argument('--extrinsics_file_path', type=str, default=AZURE_KINECT_EXTRINSICS) 
    args = parser.parse_args()

    print('Starting robot')
    fa = FrankaArm()    

    print('Opening Grippers')
    #Open Gripper
    fa.open_gripper()

    #Reset Pose
    fa.reset_pose() 
    #Reset Joints
    fa.reset_joints()

    cv_bridge = CvBridge()
    azure_kinect_intrinsics = CameraIntrinsics.load(args.intrinsics_file_path)
    azure_kinect_to_world_transform = RigidTransform.load(args.extrinsics_file_path)    

    azure_kinect_rgb_image = get_azure_kinect_rgb_image(cv_bridge)
    azure_kinect_depth_image = get_azure_kinect_depth_image(cv_bridge)

    object_image_position = np.array([800, 800])


    tool1_z_height = 0.01  # center of tool height = 1 cm
    intermediate_pose_z_height = 0.30

    '''
    Initial pose -> tool pose
    Tools pose on the table are preprocessed to be the correct pose for gripper to move
    '''
    topic_name = "/aruco_multiple/pose"+ str(tool_id)
    #pose1 = rospy.wait_for_message("/aruco_multiple/pose1", Pose) #grab pose of aruco marker tool 1
    pose1 = rospy.wait_for_message(topic_name, Pose) #grab pose of aruco marker tool_id 
    
    rot_matrix = quaternion_rotation_matrix(pose1.orientation.x, pose1.orientation.y,pose1.orientation.z, pose1.orientation.w)
    # rotate about y axis 180 deg (change z axis to point down)
    rot_grip_flip = np.array([[-1, 0, 0],[0,1,0],[0, 0, -1]])
    tool_rotation = rot_matrix@rot_grip_flip  # right hand mulplication
    
    # transform tool pose to the world frame
    tool1_transformed =  azure_kinect_to_world_transform * RigidTransform(
        rotation = tool_rotation,
        translation= np.array([pose1.position.x, pose1.position.y, pose1.position.z]),
        from_frame='franka_tool', to_frame='azure_kinect_overhead'
    )

    intermediate_robot_pose1 = tool1_transformed.copy()
    intermediate_robot_pose1.translation = [tool1_transformed.translation[0], tool1_transformed.translation[1], intermediate_pose_z_height]


    # The goal pose definition : rotation, translation relative to camera frame
    goal_rotation = np.array([[0, 0, 1],
                            [0, -1, 0],
                            [1, 0, 0]])
    
    # transform goal pose to the world frame
    goal_translations = [np.array([0.375, -0.33, 0.26]), np.array([0.375, -0.245, 0.26]), np.array([0.375, -0.16, 0.26]), np.array([0.375, -0.075, 0.26])]
    goal_transformed =  azure_kinect_to_world_transform * RigidTransform(
        rotation = goal_rotation,
        # translation= np.array([0.375, -0.33, 0.26]),  # hard coded the goal translation relative to camera_frame 1
        # translation= np.array([0.375, -0.245, 0.26]),  # hard coded the goal translation relative to camera_frame 2
        # translation= np.array([0.375, -0.16, 0.26]),  # hard coded the goal translation relative to camera_frame 3
        # translation= np.array([0.375, -0.075, 0.26]),  # hard coded the goal translation relative to camera_frame 4

        translation = goal_translations[available_pegboard_spot],
        from_frame='franka_tool', to_frame='azure_kinect_overhead'
    )

   
####################################################

    #Move to intermediate robot pose aruco 1
    fa.goto_pose(intermediate_robot_pose1)

    # use hard coded tool 1 height
    tool1_transformed.translation[2] = tool1_z_height    

    #Move to tool 1
    fa.goto_pose(tool1_transformed, 5, force_thresholds=[10, 10, 10, 10, 10, 10])

    #Close Gripper
    fa.goto_gripper(0.03, grasp=True, force=10.0)

    #Move to intermediate robot pose aruco 1 (higher than the pegboard)
    intermediate_robot_pose1.translation[2] = 0.55
    fa.goto_pose(intermediate_robot_pose1)    

    # move to the new intermediate robot pose on the pegboard
    intermediate_robot_pose1_new = goal_transformed.copy()
    intermediate_robot_pose1_new.translation[2] = 0.55
    fa.goto_pose(intermediate_robot_pose1_new)
    
    # move to the goal pose
    goal_transformed.translation[2] = 0.265  # goal height in the world frame. This is to fix the z axis to the pegboard dropoff height.
    fa.goto_pose(goal_transformed, 5, force_thresholds=[10, 10, 10, 10, 10, 10])
    print('Opening Grippers')
    #Open Gripper
    fa.open_gripper()

    #Move to intermediate robot pose 
    fa.goto_pose(intermediate_robot_pose1_new)
    fa.goto_pose(intermediate_robot_pose1)

####################################################

    #Reset Pose
    fa.reset_pose() 
    #Reset Joints
    fa.reset_joints()

if __name__ == '__main__':
    run_pick_up(0,4)