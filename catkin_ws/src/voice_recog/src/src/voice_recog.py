#! /usr/bin/env python

import os
from pocketsphinx import Decoder
import pyaudio
import math
import enum

import rospy
from std_msgs.msg import Int32


class Command(enum.Enum):
    STOP = 1
    TARGET = 2
    HOME = 3

triggerlist = ['coborg']
stoplist = ['stop']
targetlist = ['target','take','goal']
homelist = ['home','compact']

voice_dir = '/home/coborg/Coborg-Platform/.voice_subsystem/'
model_dir = '/home/coborg/Coborg-Platform/.voice_subsystem/model'


# Init decoder
config = Decoder.default_config()
config.set_string('-hmm', os.path.join(model_dir, 'en-us'))
config.set_string('-lm', os.path.join(model_dir, 'en-us.lm.bin'))
config.set_string('-dict', os.path.join(model_dir, 'coborg-cmudict-en-us.dict'))
config.set_string('-logfn', '/dev/null')
decoder = Decoder(config)

# Add searches
decoder.set_kws('coborg',os.path.join(voice_dir, 'src/keywords.txt'))
decoder.set_lm_file('lm', os.path.join(model_dir, 'en-us.lm.bin'))
decoder.set_search('coborg')

# Start listening
p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)
stream.start_stream()

# Initialize ROS
rospy.init_node('voice_recog')
voice_commands_pub = rospy.Publisher('/voice_commands', Int32, queue_size=1)

command = False
in_speech_bf = False
decoder.start_utt()
while not rospy.is_shutdown():
    command = False
    buf = stream.read(1024)
    if buf:
        decoder.process_raw(buf, False, False)
        if decoder.get_in_speech() != in_speech_bf:
            in_speech_bf = decoder.get_in_speech()
            if not in_speech_bf:
                decoder.end_utt()

                # Print hypothesis and switch search to another mode
                print('Decoder Mode:', decoder.get_search())
                results = [seg.word for seg in decoder.seg()]
                results[:] = [word for word in results if word != '<sil>' and word != '</s>' and word != '<s>']
                

                # If in "Command Mode" (after 'coborg' is heard), check for command
                if decoder.get_search() == 'lm':
                    print ('Result:', results)
                    if any(word in stoplist for word in results):
                        print(repr(Command.STOP))
                        voice_commands_pub.publish(Command.STOP.value)
                        os.system('mpg123 -q ' + voice_dir + 'Sounds/stopSound.mp3')
                        command = True
                    elif any(word in targetlist for word in results):
                        print(repr(Command.TARGET))
                        voice_commands_pub.publish(Command.TARGET.value)
                        os.system('mpg123 -q ' + voice_dir + 'Sounds/commandSound.mp3')
                        command = True
                    elif any(word in homelist for word in results):
                        print(repr(Command.HOME))
                        voice_commands_pub.publish(Command.HOME.value)
                        os.system('mpg123 -q ' + voice_dir + 'Sounds/commandSound.mp3')
                        command = True
                    elif 'gas' in results:
                        os.system('mpg123 -q ' + voice_dir + 'Sounds/fartSound.mp3')
                        command = True
                    elif 'drop' in results and 'beat' in results:
                        os.system('mpg123 -q ' + voice_dir + 'Sounds/sickbeatSound.mp3')
                        command = True
                    elif 'sarah' in results and 'hi' in results:
                        os.system('mpg123 -q ' + voice_dir + 'Sounds/rickroll.mp3')
                        command = True

                # Send stop command when "stop" is heard 3 or more times outside of "Coborg" trigger
                if any(word in stoplist for word in results):
                        if results.count('stop') > 4:
                            print(repr(Command.STOP))
                            voice_commands_pub.publish(Command.STOP.value)
                            os.system('mpg123 -q ' + voice_dir + 'Sounds/stopSound.mp3')
                
                # Translate to base language model if 'coborg' is heard.
                # Switch back to trigger model if language model hears a command (plays failure sound if command not valid)
                if any(word in triggerlist for word in results):
                    os.system('mpg123 -q ' + voice_dir + 'Sounds/triggerSound.mp3')
                    decoder.set_search('lm')
                elif decoder.get_search() == 'lm' and len(results) > 0:
                     decoder.set_search('coborg')
                     if not command:
                         os.system('mpg123 -q ' + voice_dir + 'Sounds/nocommandSound.mp3')
                    

                decoder.start_utt()
    else:
        break
decoder.end_utt()
