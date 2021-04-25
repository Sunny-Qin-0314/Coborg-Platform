from frankapy import FrankaArm
import numpy as np
import argparse
import cv2
from cv_bridge import CvBridge
from autolab_core import RigidTransform, Point
from perception import CameraIntrinsics
from utils import *
import rospy
from geometry_msgs.msg import Pose

AZURE_KINECT_INTRINSICS = 'calib/azure_kinect.intr'
AZURE_KINECT_EXTRINSICS = 'calib/azure_kinect_overhead/azure_kinect_overhead_to_world.tf'
block_height = 0.042

def get_aruco_pose(topic='/aruco_simple/pose'):
    pose_msg = rospy.wait_for_message(topic, Pose)
    return pose_msg.position, pose_msg.orientation

def get_aruco_pose2(topic='/aruco_simple/pose2'):
    pose_msg = rospy.wait_for_message(topic, Pose)
    return pose_msg.position, pose_msg.orientation

def get_block_center_point_in_world(position, transform):
    T_ee_rot = RigidTransform(
        translation=np.array([position.x, position.y, position.z]),
        from_frame='franka_tool', to_frame='azure_kinect_overhead'
    )
    object_center_point_in_world = transform * T_ee_rot
    return object_center_point_in_world

if __name__ == '__main__':
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

    def onMouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
           print('x = %d, y = %d'%(x, y))
           param[0] = x
           param[1] = y
        
    cv2.namedWindow('image')
    cv2.imshow('image', azure_kinect_rgb_image)
    cv2.setMouseCallback('image', onMouse, object_image_position)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    object_z_height = 0.02
    intermediate_pose_z_height = 0.19

    object_center_point_in_world = get_object_center_point_in_world(object_image_position[0],
                                                                    object_image_position[1],
                                                                    azure_kinect_depth_image, azure_kinect_intrinsics,
                                                                    azure_kinect_to_world_transform)
    object_center_pose = fa.get_pose()
    object_center_pose.translation = [object_center_point_in_world[0], object_center_point_in_world[1], object_z_height]

        
    # new_rotation = np.array([[np.cos(theta), -np.sin(theta), 0],
    #                       [-np.sin(theta), -np.cos(theta), 0],
    #                       [0, 0, -1]])
    # object_center_pose.rotation = new_rotation


    intermediate_robot_pose = object_center_pose.copy()
    intermediate_robot_pose.translation = [object_center_point_in_world[0], object_center_point_in_world[1], intermediate_pose_z_height]

    block1position_rel, block1orientation = get_aruco_pose()
    block2position_rel, block2orientation = get_aruco_pose2()
    block1position = get_block_center_point_in_world(block1position_rel, azure_kinect_to_world_transform)
    block2position = get_block_center_point_in_world(block2position_rel, azure_kinect_to_world_transform)

    block1_robot_pose = block1position.copy()
    block1_robot_pose.translation[2] = intermediate_pose_z_height
    block1_object_robot_pose = block1position.copy()
    block1_object_robot_pose.translation[2] = object_z_height
    block1_target_robot_pose = object_center_pose.copy()
    block1_target_robot_pose.translation[2] = object_z_height+block_height

    block2_robot_pose = block2position.copy()
    block2_robot_pose.translation[2] = intermediate_pose_z_height
    block2_object_robot_pose = block2position.copy()
    block2_object_robot_pose.translation[2] = object_z_height
    block2_target_robot_pose = object_center_pose.copy()
    block2_target_robot_pose.translation[2] = object_z_height+2*block_height

    # Get Block 1 ----------------------------------------------------------------------------------------------------------
    #Move to intermediate robot pose
    fa.goto_pose(block1_robot_pose)

    fa.goto_pose(block1_object_robot_pose, 5, force_thresholds=[10, 10, 10, 10, 10, 10])

    #Close Gripper
    fa.goto_gripper(0.045, grasp=True, force=10.0)

    # Move to block intermediate
    fa.goto_pose(block1_robot_pose)
    
    #Move to intermediate robot pose
    fa.goto_pose(intermediate_robot_pose)

    fa.goto_pose(block1_target_robot_pose, 5, force_thresholds=[10, 10, 20, 10, 10, 10])

    fa.open_gripper()
    fa.goto_pose(intermediate_robot_pose)

    #Get Block 2 ----------------------------------------------------------------------------------------------------------
    #Move to intermediate robot pose
    fa.goto_pose(block2_robot_pose)

    fa.goto_pose(block2_object_robot_pose, 5, force_thresholds=[10, 10, 10, 10, 10, 10])

    #Close Gripper
    fa.goto_gripper(0.045, grasp=True, force=10.0)

    # Move to block intermediate
    fa.goto_pose(block2_robot_pose)
    
    #Move to intermediate robot pose
    fa.goto_pose(intermediate_robot_pose)

    fa.goto_pose(block2_target_robot_pose, 5, force_thresholds=[10, 10, 20, 10, 10, 10])

    fa.open_gripper()
    fa.goto_pose(intermediate_robot_pose)

    #Reset Pose
    fa.reset_pose() 
    #Reset Joints
    fa.reset_joints()