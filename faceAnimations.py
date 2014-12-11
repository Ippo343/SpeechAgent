#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('sound_play')
import roslib; roslib.load_manifest('syntheligence')
from syntheligence.srv import *
from syntheligence.msg import *


# Standard durations
d0 = 0.001
d1 = 0.1
d2 = 0.2
d3 = 0.4
d4 = 0.8


# ACTIONS: elementary actions of one only muscle (or pair for symmetrical muscles)
AU_OpenEyes  = [ au("05", 0.75, d1) ]
AU_CloseEyes = [ au("05", 0   , d1) ]
AU_HeadUp    = [ au("TX", 0.05, d4) ]
AU_HeadDown  = [ au("TX", 0   , d4) ]


# SEQUENCES must be evaluated one action after the other, for example in blinking.
# Use faceTools.faceSequence to run them
SEQ_Blink = [ AU_CloseEyes, AU_OpenEyes ]


# COMMANDS must be evaluated all at the same time to bring the face to a new state, for example for smiling.
# Use faceTools.faceCommand to run them


# Set ALL parameters to zero.
# Useful between commands to avoid merging two faces
# Called automatically by faceCommand
CMD_Zero = [    au("01", 0, d0), 
                au("02", 0, d0), 
                au("03", 0, d0), 
                au("04", 0, d0), 
                au("05", 0, d0), 
                au("06", 0, d0), 
                au("07", 0, d0), 
                au("08", 0, d0), 
                au("09", 0, d0), 
                au("10", 0, d0), 
                au("11", 0, d0), 
                au("12", 0, d0), 
                au("13", 0, d0), 
                au("14", 0, d0), 
                au("15", 0, d0), 
                au("16", 0, d0), 
                au("17", 0, d0), 
                au("18", 0, d0), 
                au("19", 0, d0), 
                au("20", 0, d0), 
                au("21", 0, d0), 
                au("22", 0, d0), 
                au("23", 0, d0), 
                au("24", 0, d0), 
                au("25", 0, d0), 
                au("26", 0, d0), 
                au("27", 0, d0), 
                au("28", 0, d0), 
                au("29", 0, d0), 
                au("30", 0, d0), 
                au("31", 0, d0), 
                au("32", 0, d0), 
                au("33", 0, d0), 
                au("34", 0, d0), 
                au("35", 0, d0), 
                au("36", 0, d0), 
                au("37", 0, d0), 
                au("38", 0, d0), 
                au("39", 0, d0), 
                au("SX", 0, d0), 
                au("TX", 0, d0), 
                au("TY", 0, d0), 
                au("TZ", 0, d0) ]
                
                               
# Neutral face
CMD_Neutral = CMD_Zero + [  au("01", 0.4, d2),
                            au("02", 0.6, d2),
                            au("05", 0.8, d2), 
                            au("06", 0.0, d2), 
                            au("12", 0.4, d2),
                            au("13", 0.4, d2),
                            au("14", 0.4, d2)] 


# Serious face
CMD_Serious = CMD_Zero + [  au("02", 0.8, d2),
                            au("04", 0.2, d2), 
                            au("05", 0.8, d2), 
                            au("09", 0.5, d2),
                            au("12", 0.3, d2),
                            au("13", 0.3, d2),
                            au("15", 0.4, d2)] 

# Happy smiling face
CMD_Happy = CMD_Zero + [au("01", 0.7, d2),
                        au("02", 1.0, d2),
                        au("05", 0.9, d2), 
                        au("06", 1.0, d2), 
                        au("07", 0.4, d2), 
                        au("12", 1.0, d2),
                        au("13", 0.4, d2),
                        au("14", 0.5, d2)]

# Scared face          
CMD_Help = CMD_Zero + [ au("01", 1.0, d1),
                        au("05", 1.0, d1),
                        au("10", 0.4, d1), 
                        au("15", 0.2, d1), 
                        au("16", 0.5, d1), 
                        au("18", 0.3, d1),
                        au("25", 0.2, d1),
                        au("26", 0.3, d1),
                        au("27", 0.4, d1),
                        au("38", 1.0, d1)]

