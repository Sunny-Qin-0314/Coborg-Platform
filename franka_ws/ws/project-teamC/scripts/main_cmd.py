#! /usr/bin/env python
import enum
import os

from utils import *
from rospy.exceptions import ROSException
from frankapy import FrankaArm
from geometry_msgs.msg import Pose

import time
import math
from pynput import keyboard
import numpy as np

import params
import pick
import place


fa =FrankaArm() 

class Command(enum.IntEnum): #these could be voice commands. if onlyyy gerrryyy2 was cooooooool.
    ERROR = 0
    PICK = 1 
    PLACE = 2 

class Tool(enum.IntEnum): #tool type
    ERROR = 0
    PHILIP_SHORT = 1
    FLAT_SHORT = 2
    PHILIP_LONG = 3
    FLAT_LONG = 4 
    ALL = 9

def prompt():
    command_input = input('Pick(1) or Place(2)?').upper() #convert input to upper case
    tool_input = input('Tool(1), Tool(2), Tool(3), Tool(4), All(9)?').upper()

    #auto selector for picking
    if tool_input == '9' or tool_input == 'ALL': #if pick/place all was called
        if command_input == '1': #if pick
            pegboard_tool_idx = np.where(np.array(params.pegboard) > 0)[0]
            for tool_idx in pegboard_tool_idx:
                new_command(command_input, tool_idx+1)
        elif command_input == '2': #if place
            timeout = time.time() + 60*5 #run for 5 minutes
            while time.time() < timeout and params.available:
                
               for tag in params.available:
                    try: #if there's data in the tag
                        topic_name = "/aruco_multiple/pose"+ str(tag)
                        pose = rospy.wait_for_message(topic_name, Pose, timeout=0.1)  
                        new_command(command_input, tag)
                        params.available = [tool for tool in range(1,5) if tool not in params.pegboard] #update what's available to pick
                    except ROSException: #if the "wait for message" times out then pass to next tag
                        print("searching...")
                        
    else:
        new_command(command_input, tool_input)


def new_command(command_input, tool_input):
    
    if command_input.isdigit() or tool_input.isdigit(): #if you're lazy and use numbers
        try:
            command = Command(int(command_input)) #cast to int
            tool = Tool(int(tool_input))
        except ValueError: #if your spelling is garbage
            command = Command.ERROR
            tool = Tool.ERROR
    else:
        try:
            command = Command[command_input] #convert message to command type
            tool = Tool[tool_input]
        except KeyError: #if your spelling is still garbage
            command = Command.ERROR
            tool = Tool.ERROR   

    #check what command was sent and execute
    if command == Command.PICK:
        pick.execute(tool) #run picking function
        print(params.pegboard) #display what is on the pegboard

    elif command == Command.PLACE:
        place.execute(tool)
        print(params.pegboard) #display what is on the pegboard
        
    else:
        print("Error: Input Not Found")

if __name__ == "__main__":
    params.validate() #define where everything is on startup
    print("Current pegboard status = {}\n".format(params.pegboard))
    while True:
        prompt()