# !/usr/bin/env python3
import enum
import os
from pocketsphinx import Decoder
import pyaudio
import math

import params
import pick
import place

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

### VOICE COMMAND ADDITION SETUP (Optional)
triggerlist = ['frank']
commandlist = ['pick','place']
toollist = ['one','one(2)','two','too','to','three','four','for','all']

franka_dir = '/home/coborg/Coborg-Platform/franka_ws/ws/project-teamC/scripts'
model_dir = '/home/coborg/Coborg-Platform/catkin_ws/src/voice_recog/src/model'

# Init decoder
config = Decoder.default_config()
config.set_string('-hmm', os.path.join(model_dir, 'en-us'))
config.set_string('-lm', os.path.join(model_dir, 'en-us.lm.bin'))
config.set_string('-dict', os.path.join(model_dir, 'franka.dict'))
config.set_string('-logfn', '/dev/null')
decoder = Decoder(config)

# Add searches
decoder.set_kws('franka',os.path.join(franka_dir, 'keywords.txt'))
decoder.set_lm_file('lm', os.path.join(model_dir, 'en-us.lm.bin'))
decoder.set_search('franka')
print(decoder.get_kws('franka'))

# Start listening
p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)
stream.start_stream()
### END VOICE COMMAND SETUP

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
    print("Current pegboard status = {}".format(params.pegboard))

    # while True:

    #     command_input = input('Pick(1) or Place(2)? Ctrl+C to quit.').upper() #convert input to upper case
    #     tool_input = input('Tool(1), Tool(2), Tool(3), Tool(4), All(9)?').upper()
    #     new_command(command_input, tool_input)

    ### VOICE COMMAND ADDITION LOOP (Optional) 
    command = False
    in_speech_bf = False
    decoder.start_utt()
    while True:
        command = False
        buf = stream.read(1024)
        if buf:
            # Send raw audio to pocketsphinx decoder
            decoder.process_raw(buf, False, False)
            # Once speech is detected, keep listening until no more speech is detected before processing command.
            if decoder.get_in_speech() != in_speech_bf:
                in_speech_bf = decoder.get_in_speech()
                # Once speech is completed (decoder.get_in_speech() is set back to false), process phrase
                if not in_speech_bf:
                    decoder.end_utt()

                    # Print hypothesis and switch search to another mode
                    print('Decoder Mode:', decoder.get_search())
                    results = [seg.word for seg in decoder.seg()]
                    results[:] = [word for word in results if word != '<sil>' and word != '</s>' and word != '<s>']
                    

                    # If in "Command Mode" (after 'coborg' is heard), check for command
                    if decoder.get_search() == 'lm':
                        print ('Result:', results)
                        if any(word in commandlist for word in results):
                            commands = [word for word in results if word in commandlist]
                            print(commands[0])
                            command = '0'
                            if 'pick' in commands[0]:
                                command = '1'
                            if 'place' in commands[0]:
                                command = '2'
                            if any(word in toollist for word in results):
                                print("Found a tool")
                                tools = [word for word in results if word in toollist]
                                print(command[0],tools[0])
                                tool = '0'
                                if 'one' in tools[0]:
                                    tool = '1'
                                if 'too' in tools[0] or 'two' in tools[0] or 'to' in tools[0]:
                                    tool = '2'
                                if 'three' in tools[0]:
                                    tool = '3'
                                if 'four' in tools[0] or 'for' in tools[0]:
                                    tool = '4'
                                new_command(command, tool)
                    
                    # Translate to base language model if 'coborg' is heard.
                    # Switch back to trigger model if language model hears a command (plays failure sound if command not valid)
                    if any(word in triggerlist for word in results):
                        # os.system('mpg123 -q ' + model_dir + '/../Sounds/triggerSound.mp3')
                        decoder.set_search('lm')
                    elif decoder.get_search() == 'lm' and len(results) > 0:
                        decoder.set_search('franka')
                        # if not command:
                        #     os.system('mpg123 -q ' + model_dir + '/../Sounds/nocommandSound.mp3')
                        

                    decoder.start_utt()
        else:
            break
    decoder.end_utt()
    ### END VOICE COMMAND LOOP