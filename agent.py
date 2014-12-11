#!/usr/bin/env python

# Agent node that executes the feedback actions (speaking etc) associated to the pose notifications.


################# IMPORTS

# ROS libraries: rospy, sound_play and syntheligence
import rospy
from std_msgs.msg import *
import roslib;
roslib.load_manifest('sound_play')
roslib.load_manifest('syntheligence')

# Speech synthesis
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

# Face animations
from syntheligence.srv import *
from syntheligence.msg import *
from faceTools import *         # helper functions
from faceAnimations import *    # animation commands

# Pose notification type
from pose_detection.msg import PoseNotification

# Shared names for topics and nodes
from globalNames import *

# To randomize the blinking
import random

########################

loopRate = 60

reactTime = rospy.Duration(0.5) # reaction time

speaker = ()                    # handle to the sound_play server
voice = 'voice_rab_diphone'     # voice for the sound_play server

faceServer = ()                 # handle to the syntheligence server


# Variables to control the blinking
lastBlink = ()
blinkWait = ()
blinkMin  = 0.5
blinkMax  = 8.0

# Resets the state of the blinking mechanism
def resetBlink():
    
    global lastBlink, blinkWait

    lastBlink = rospy.Time.now()
    blinkWait = rospy.Duration(random.uniform(blinkMin, blinkMax))
    
    
# Variables to control the animation
lastAnim = ()
animTime = rospy.Duration(3)


#### "State" of the agent (to time correctly the reactions)

# last matched pose (of type PoseNotification, stores the received message)
# if the same pose is received twice, this is not updated: this way we can measure how long the pose has been held before we trigger the reaction
savedPose = PoseNotification( std_msgs.msg.Header(), '', '', '')

# Flag to check if the reaction has already been triggered (avoid double activation)
reacted = False



# Speaks a phrase through the speaking server
def sayPhrase(p):

    print 'Saying: %s' % p  
    speaker.say(p, voice)
    


################# AGENT LOGIC

def notificationCallback(msg):

    global savedPose
    global lastAnim, reacted
    global lastBlink

    # If the user changes pose, we don't react
    # The new pose is stored (along with the timestamp) and the reaction will only start after the specified reaction time
    # The check is run only on the name: in fact the two messages will always be 'different' because they contain different timestamps
    if not (msg.Name == savedPose.Name):
    
        print "Pose changed to %s" % msg.Name
        savedPose = msg;
        reacted = False   # you need to re-enter a pose to re-trigger the reaction
        
        return

       
    # If we didn't return, it meas that the user is holding the pose:
    # react accordingly to the message, after checking the reaction time.
    # If the reaction has already been triggered, do nothing
    
    
    # Not reacted, pose has been held longer that the reaction time: play the animation
    if (not reacted) and ( (msg.MsgHeader.stamp - savedPose.MsgHeader.stamp) > reactTime ):
    
        # set the flag
        reacted = True
    
        # Audio feedback
        if not (msg.Phrase == ''):
            sayPhrase(msg.Phrase)
        
        # Visual feedback
        if not msg.Command == '':
        
            lastAnim = rospy.Time.now()
        
            # Differentiate between commands (all at once) and sequences of movements
            if msg.Command[0:3] == 'CMD':
                faceCommand(faceServer, eval(msg.Command))
            elif msg.Command[0:3] == 'SEQ':
                faceSequence(faceServer, eval(msg.Command))
            else:
                raise error('Unrecognizable face command, aborting!')
                
        else:            
            resetBlink()
                
    
        return    




############ MAIN LOOP:
# initializes the necessary nodes and services and then spins waiting for notifications

if __name__ == '__main__':
    
    # Start the agent node
    rospy.init_node(agentNode, anonymous = False)
    
    # Start the speaker and wait for its initialization
    speaker = SoundClient(); rospy.sleep(1)
    
    # Wait for the face bridge service to be available
    rospy.wait_for_service(faceService)
    
    # Get the handle to the server, clear the face and wait for the initialization
    faceServer = rospy.ServiceProxy(faceService, SendAction)
    faceCommand( faceServer, CMD_Zero )
    rospy.sleep(2)
    
    # Get the face into a neutral expression
    lastAnim = rospy.Time.now()
    faceCommand(faceServer, CMD_Neutral)
    
    # We are now ready to receive the notifications, subscribe to the topic
    rospy.Subscriber(notificationTopic, PoseNotification, notificationCallback)
    
    # Audio acknowledge
    sayPhrase('Ready.')
    
    # Reset the blinking system
    resetBlink()
    
    
    # Blinking loop 
    rateChecker = rospy.Rate(loopRate)
    while not rospy.is_shutdown():

        if (rospy.Time.now() - lastBlink) > blinkWait:
            # Blink and choose another random interval
            faceSequence( faceServer, SEQ_Blink )
            lastBlink = rospy.Time.now()
            blinkWait = rospy.Duration(random.uniform(blinkMin, blinkMax))
   
        rateChecker.sleep() # force the execution rate   
    
    
   
