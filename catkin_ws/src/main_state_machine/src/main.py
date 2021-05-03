#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Char
import enum
import pyaudio
from pydub import AudioSegment
from pydub.playback import play

# Updates List:
# Consider changing the state machine to work on actions instead of publisher/subscriber
# Figure out how to publish sounds to speaker

# Functions:
# 1 = e_stop > Shut off power to motors
# 2 = hold > Move to and hold plate
# 3 = compact > Return to home position
# # = caution > Motors stop moving, but maintain with lower torque threshold

class Command(enum.IntEnum): #these are the voice commands that come in and get sent to the motors
    STOP = 1 
    TARGET = 2 
    HOME = 3
    READY = 4

# Statuses:
# 1 = initializing > Command received, but not executing yet (e.g. detecting hands) [ROS INFO]
# 2 = executing > Command being executed (e.g. moving to target)
# 3 = waiting > Command completed/performing holding task, ready for next command (maintaining position in 3d space)

class Status(enum.IntEnum): #this is the status that outputs to ROS terminal
    INIT = 1 
    EXECUTE = 2 
    IDLE = 3

# Speaker Sounds (to be implemented):
# 0 = start-up > System has launched
# 1 = listening > Keyword "COBORG" identified, awaiting full command
# 2 = understood > Voice command understood
# 3 = not understood > Voice command not understood
# 4 = emergency > Emergency command detected
# 5 = waiting > Task completed, available for next command

status = Status.INIT # initializing
function = Command.HOME  # compact robot arm mode

status = Status.IDLE #idle/ready

# Function for /voice_commands
def new_command(message):
    new_command = message.data
    global status, function, state_output_pub
    print("COMMAND RECEIVED:")

    if new_command == Command.STOP:
        print("STOP")
        function = Command.STOP # e_stop
        status = Status.EXECUTE # execute mode
        state_output_pub.publish(function)

    elif new_command == Command.TARGET:
        print("TARGET")
        if status == Status.IDLE: #idle/ready
            function = Command.TARGET # hold
            status = Status.EXECUTE # execute mode
            state_output_pub.publish(function)

    elif new_command == Command.HOME:
        print("HOME")
        if status == Status.IDLE: #idle/ready
            function = Command.HOME # compact
            status = Status.EXECUTE # execute mode
            state_output_pub.publish(function)
    
    elif new_command == Command.READY:
        print("READY")
        if status == Status.IDLE: #idle/ready
            function = Command.READY # compact
            status = Status.EXECUTE # execute mode
            state_output_pub.publish(function)
    """
    elif new_command == Command.STOP.value:
        if function != 0:
            function = 1 # caution
            status = 'a' # initializing
            state_output_pub.publish(function)
    """

# Function for /state_input
def status_update(message):
    new_status = message.data
    global status
    if new_status == Status.EXECUTE:
        status = Status.EXECUTE # executing
    if new_status == Status.IDLE:
        status = Status.IDLE # waiting
        #speaker_output_pub.publish(5)


if __name__ == "__main__":
    # Initializations
    rospy.init_node('main_state_machine')
    voice_commands_sub = rospy.Subscriber('/voice_commands', Int32, new_command)
    state_output_pub = rospy.Publisher('/state_output', Int32, queue_size=1)
    #speaker_output_pub = rospy.Publisher('/speaker_output', Int32, queue_size=1)
    state_input_sub = rospy.Subscriber('/state_input', Int32, status_update)
    state_output_pub.publish(function)
    #speaker_output_pub.publish(0)

    rospy.spin() # Should this just be spin()?

