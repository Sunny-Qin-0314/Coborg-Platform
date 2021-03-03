#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Char
import enum

# Updates List:
# Consider changing the state machine to work on actions instead of publisher/subscriber
# Figure out how to publish sounds to speaker

# Functions:
# 1 = e_stop > Shut off power to motors
# 2 = hold > Move to and hold plate
# 3 = compact > Return to home position
# # = caution > Motors stop moving, but maintain with lower torque threshold

class Command(enum.Enum):
    STOP = 1
    TARGET = 2
    HOME = 3

# Statuses:
# a = initializing > Command received, but not executing yet (e.g. detecting hands)
# b = executing > Command being executed (e.g. moving to target)
# c = waiting > Command completed/performing holding task, ready for next command (maintaining position in 3d space)

# Speaker Sounds:
# 0 = start-up > System has launched
# 1 = listening > Keyword "COBORG" identified, awaiting full command
# 2 = understood > Voice command understood
# 3 = not understood > Voice command not understood
# 4 = emergency > Emergency command detected
# 5 = waiting > Task completed, available for next command

status = 'c' # initializing
function = 2 # compact

# Function for /voice_commands
def new_command(message):
    new_command = message.data
    global status, function, state_output_pub
    print("COMMAND RECEIVED:")
    if new_command == Command.STOP.value:
        print("STOP")
        function = Command.STOP.value # e_stop
        status = 'c' # initializing
        state_output_pub.publish(function)
    elif new_command == Command.TARGET.value:
        print("TARGET")
        if status == 'c':
            function = Command.TARGET.value # hold
            status = 'c' # initializing
            state_output_pub.publish(function)
    elif new_command == Command.HOME.value:
        print("HOME")
        if status == 'c':
            function = Command.HOME.value # compact
            status = 'c' # initializing
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
    if new_status == 'b':
        status = 'b' # executing
    if new_status == 'c':
        status = 'c' # waiting
        #speaker_output_pub.publish(5)

# Initializations
rospy.init_node('main_state_machine')
voice_commands_sub = rospy.Subscriber('/voice_commands', Int32, new_command)
state_output_pub = rospy.Publisher('/state_output', Int32, queue_size=1)
#speaker_output_pub = rospy.Publisher('/speaker_output', Int32, queue_size=1)
#state_input_sub = rospy.Subscriber('/state_input', Char, status_update)
state_output_pub.publish(function)
#speaker_output_pub.publish(0)

rospy.spin() # Should this just be spin()?

