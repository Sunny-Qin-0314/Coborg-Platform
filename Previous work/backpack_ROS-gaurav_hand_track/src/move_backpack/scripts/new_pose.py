#!/usr/bin/env python

import varsList as vL
import pygame
import rospy
import os
from sensor_msgs.msg import JointState

# Init joystick, pygame, and ros modules
pygame.init()
pygame.joystick.init()
pygame.display.set_mode((1, 1))

cur_pose = [0]*3

def feedback(data):
    global cur_pose
    cur_pose = data.position

rospy.init_node("learning")
rospy.Subscriber("/hebiros/right_arm/feedback/joint_state", JointState, feedback)

# When 12th button is pressed, goes to inLearning mode
inLearning = False

positions = [0]*12
for i in range(12):
    positions[i] = [0, 0, 0, 0]

for x in range(pygame.joystick.get_count()):
    pygame.joystick.Joystick(x).init()

# Update text file with new poses; pose 1,2,3,4 are fixed
def update(positions):
    content = ""
    for x in range(12):
        if x == 0:
            content += "1" + " " + str(1.2512) + " " + str(-0.7863) + \
            " " + str(-0.8820) + "\n"
        elif x == 1:
            content += "1" + " " + str(1.0225) + " " + str(-1.4381) + \
            " " + str(-1.3667) + "\n"
        elif x == 2:
            content += "1" + " " + str(1.0734) + " " + str(-0.7035) + \
            " " + str(-0.7844) + "\n"
        elif x == 3:
            content += "1" + " " + str(-2.4394) + " " + str(-2.4953) + \
            " " + str(-2.5588) + "\n"
        else:
            content += str(positions[x][0]) + " " + str(positions[x][1]) + " " + \
            str(positions[x][2]) + " " + str(positions[x][3]) + "\n"
    return content

while True: 
    for event in pygame.event.get():
	pass
    
    for ax in range(pygame.joystick.Joystick(0).get_numaxes()):
        vL.axes[ax] = pygame.joystick.Joystick(0).get_axis(ax)
    for button in range(pygame.joystick.Joystick(0).get_numbuttons()):
        vL.buttons[button] = pygame.joystick.Joystick(0).get_button(button)
    for hat in range(pygame.joystick.Joystick(0).get_numhats()):
        vL.povs[hat] = pygame.joystick.Joystick(0).get_hat(hat)
   
    # Update new poses when in Learning mode
    if vL.buttons[11] or inLearning:
        if not inLearning:
            print("in Learning mode")
        inLearning = True
        for pose in range(12):
            if vL.buttons[pose] and pose > 3 and pose != 11:
                positions[pose][0] = "1"
                positions[pose][1] = round(cur_pose[0], 4)
                positions[pose][2] = round(cur_pose[1], 4)
                positions[pose][3] = round(cur_pose[2], 4)
                content = update(positions)
                print("button %d pressed!" % (pose+1))
                with open("all_poses.txt", "wt") as f:
                    f.write(content)
                inLearning = False
        
    pygame.time.wait(10)


    	



