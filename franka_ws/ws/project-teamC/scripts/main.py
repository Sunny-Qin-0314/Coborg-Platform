#! /usr/bin/env python
import enum

import params
import pick
import place

class Command(enum.IntEnum): #these could be voice commands. if onlyyy gerrryyy2 was cooooooool.
    ERROR = 0
    PICK = 1 
    PLACE = 2 

class Tool(enum.IntEnum): #tool type
    ERROR = 0
    JONATHAN = 1
    GERRY = 2
    YUQING = 3
    JASON = 4 
    ALL = 9

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
        #print ('Picking', tool.name)
        pick.execute(tool) #run picking function
        print(params.pegboard) #display what is on the pegboard

    elif command == Command.PLACE:
        #print ('Placing', tool.name)
        place.execute(tool)
        print(params.pegboard) #display what is on the pegboard
        
    else:
        print("Error: Input Not Found")

    

if __name__ == "__main__":

    params.validate() #define where everything is on startup
    print(params.pegboard)

    params.pegboard = [1,0,0,2]
    print(params.pegboard)

    while True:
        command_input = input("Pick(1) or Place(2)? Ctrl+C to quit.").upper() #convert input to upper case
        tool_input = input("Tool(1), Tool(2), Tool(3), Tool(4), All(9)?").upper()
        new_command(command_input, tool_input)

