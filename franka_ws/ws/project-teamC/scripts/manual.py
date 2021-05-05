from os import wait
from frankapy import FrankaArm
# from frankapy.utils import min_jerk, min_jerk_weight
# from autolab_core import RigidTransform, Point

# from geometry_msgs.msg import Pose
# import math
# import numpy as np

import time
import keyboard

fa = FrankaArm()
pose = fa.get_pose()
action = True

def moveArm(direction):
    if direction is 'R':
        pose.translation[0] += 0.1
        # fa.goto_pose(pose)
    elif direction is 'L':
        pose.translation[0] -= 0.1
        # fa.goto_pose(pose)
    elif direction is 'U':
        pose.translation[1] += 0.1
        # fa.goto_pose(pose)
    elif direction is 'D':
        pose.translation[1] -= 0.1
        # fa.goto_pose(pose)
    elif direction is 'S':
        pose.translation[2] -= 0.1
        # fa.goto_pose(pose)
    elif direction is 'B':
        pose.translation[2] += 0.1
        # fa.goto_pose(pose)
    elif direction is 'C':
        fa.stop_skill()
        fa.close_gripper()
    elif direction is 'O':
        fa.stop_skill()
        fa.open_gripper()
    elif direction is '!':
        fa.stop_skill()
        fa.reset_pose()


if __name__ == "__main__": #unit testing code goes here
    keyboard.add_hotkey('left', moveArm, args=('L'))
    keyboard.add_hotkey('right', moveArm, args=('R'))
    keyboard.add_hotkey('up', moveArm, args=('U'))
    keyboard.add_hotkey('down', moveArm, args=('D'))
    keyboard.add_hotkey('space', moveArm, args=('S'))
    keyboard.add_hotkey('z', moveArm, args=('C'))
    keyboard.add_hotkey('x', moveArm, args=('O'))
    keyboard.add_hotkey('r', moveArm, args=('!'))
    keyboard.add_hotkey('b', moveArm, args=('B'))

    print("Control with arrow keys for left & right, space and b for up and down")
    while True:
        time.sleep(1)
        if fa.is_skill_done() and pose != fa.get_pose():
            fa.goto_pose(pose)
            pose = fa.get_pose()