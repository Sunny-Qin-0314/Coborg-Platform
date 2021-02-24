#This is a compilation of all variables used across the cobot programs.
#Can be accessed by importing varsList and calling varsList.variable

import hebi
import math
import matplotlib
import numpy as np

#Main robot model, used across board for command and feedback, characteristics
robot = None
model = None
nModules = 0
moduleList = []
kin = None
info = None
n = 0
group_command = None
efforts = [0,0,0]

#Trajectories
trajGen = None
numWayP = 0

#Feedback
fbk = None
fkNow = None
fkcom = None
fkout = None

#Theta Values
theta = 0
th = 0
thmax = None
thmin = None

#Mode functions and gravity compensation
mode = "gravComp"
gravity = np.matrix([0,0,9.81]).T
joint_mask = []

#Pushing
directionToPush = None
pushString = ""

#Animate
ax = None
plot_fk = None
fk_list = []
xPoint = [0]
yPoint = [0]
zPoint = [0]

#Joystick
axes = None
buttons = None
povs = None

#States and Goals
numGoals = 0
goalNum = 0
newGoalNum = []
lastGoalNum = -1
positionEnd = [0,0,0]
positions = []
pos = 0
lastPos = [0,0,0]
maxWayP = 20
maxDist = 0

#Activaton
sendCommands = 1
mobile = 0
animate = 0
withGripper = 0
running = 0
remainInMode = 0