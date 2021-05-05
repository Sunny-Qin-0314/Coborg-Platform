from pathlib import Path
import argparse
from perception import CameraIntrinsics
from autolab_core import RigidTransform, Point

AZURE_KINECT_INTRINSICS = Path(__file__).parent.parent/'calib'/'azure_kinect.intr'
AZURE_KINECT_EXTRINSICS = Path(__file__).parent.parent/'calib'/'azure_kinect_overhead'/'azure_kinect_overhead_to_world.tf'

def validate():
    global pegboard #allows us to call and update pegboard like a class
    global available
    global azure_kinect_intrinsics
    global azure_kinect_to_world_transform
    global userGrab 

    userNoGrab = False
    print('Loading Camera Parameters')
    parser = argparse.ArgumentParser()
    parser.add_argument('--intrinsics_file_path', type=str, default=AZURE_KINECT_INTRINSICS)
    parser.add_argument('--extrinsics_file_path', type=str, default=AZURE_KINECT_EXTRINSICS) 
    args = parser.parse_args()
    azure_kinect_intrinsics = CameraIntrinsics.load(args.intrinsics_file_path)
    azure_kinect_to_world_transform = RigidTransform.load(args.extrinsics_file_path)  
    
    pegboard = [0,0,0,0] #init a list with 4 index
    for i in range(len((pegboard))):
        pegboard[i] = int(input('What tool is in pegboard position {}? [1, 2, 3, 4, or 0 for none]'.format(i)))
    available = [tool for tool in range(1,5) if tool not in pegboard] #what's not on the pegboard
    
if __name__ == "__main__": #unit testing code goes here
    pass