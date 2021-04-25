from frankapy import FrankaArm
import numpy as np
import time


if __name__ == "__main__":
    # Initalize variables
    distThres = 0.1
    controllerSec = 20
    waitSec = 1

    fa = FrankaArm()
    print("Franka Initialized")

    print("Resetting Arm")
    fa.reset_joints()
    fa.open_gripper()

    fa.close_gripper()
    print("Gripper Closed")

    print("Starting Controller")
    pose = fa.get_pose()
    desired_position = pose.translation
    fa.apply_effector_forces_torques(controllerSec,0,0,0, block=False)
    beginTime = time.time()
    currTime = time.time()
    diffTime = float(currTime - beginTime)

    while diffTime < controllerSec:
        pose = fa.get_pose()
        position = pose.translation
        error = desired_position - position
        if np.linalg.norm(error) > distThres:
            fa.stop_skill()
            fa.open_gripper()
            print("Gripper Opened")
            break
        currTime = time.time()
        diffTime = float(currTime - beginTime)

    print("Sleeping for {} seconds".format(waitSec))
    time.sleep(waitSec)



    time.sleep(waitSec)
    fa.reset_joints()
