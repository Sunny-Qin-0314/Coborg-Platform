#! /usr/bin/env python
import enum

import params
import pick
import place

class Command(enum.IntEnum): #these could be voice commands. if onlyyy gerrryyy was cooooooool.
    ERROR = 0
    PICK = 1 
    PLACE = 2 

class Tool(enum.IntEnum): #tool type
    ERROR = 0
    SCREWDRIVER = 1 
    HAMMER = 2
    WRENCH = 3 

# Function for /voice_commands
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
        print ('Picking', tool.name)
        pick.execute(tool) #run picking function

    elif command == Command.PLACE:
        print ('Placing', tool.name)
        place.execute(tool)

    else:
        print("Error: Tool Not Found")

if __name__ == "__main__":

    params.validate() #define where everything is on startup
    print(params.pegboard)

    while True:
        command_input = input("Pick(1) or Place(2)? Ctrl+C to quit.").upper() #convert input to upper case
        tool_input = input("Screwdriver(1), Hammer(2), Wrench(3)?").upper()
        new_command(command_input, tool_input)

