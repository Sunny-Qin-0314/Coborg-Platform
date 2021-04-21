from pathlib import Path
import argparse
from perception import CameraIntrinsics
from autolab_core import RigidTransform, Point

AZURE_KINECT_INTRINSICS = Path(__file__).parent.parent/'calib'/'azure_kinect.intr'
AZURE_KINECT_EXTRINSICS = Path(__file__).parent.parent/'calib'/'azure_kinect_overhead'/'azure_kinect_overhead_to_world.tf'

def validate():
    global pegboard #allows us to call and update pegboard like a class
    global azure_kinect_intrinsics
    global azure_kinect_to_world_transform

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


    # TO DO:
    #
    # This function is validation for what tools are currently on the pegboard.
    # We need to know what tools are in what locations on the pegboard. 
    # Return a list/array that shows tool locations in the tool holder
    # 0 = Empty, 1 = Screwdriver, 2 = Hammer, 3 = Wrench
    #
    # Example: Let's say there is a Screwdriver, Nothing, Nothing, Hammer on the pegboard.
    #
    # Ask the user: What is in the first location? '1'
    # Second location? '0'
    # Third location? '0'
    # Fourth location? '2'
    #
    # store a list with [1, 0, 0, 2] for access later
    # Now, when we call "Robot pick screwdriver" it knows there's a screwdriver in index 0.
    # Also if we say "Robot place wrench" it knows that locations 1 & 2 are empty
 
if __name__ == "__main__": #unit testing code goes here
    pass