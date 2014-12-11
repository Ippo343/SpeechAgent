#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('sound_play')
import roslib; roslib.load_manifest('syntheligence')
from syntheligence.srv import *
from syntheligence.msg import *

from faceAnimations import *


def faceCommand( sender, cmd ):
    try:
        resp = sender( CMD_Zero + cmd )
        return resp.ok
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def faceSequence( sender, cmds ):
  
  for c in cmds:
  
      try:
        sender( c )        
      except rospy.ServiceException, e:
        print "Service call failed: %s" % e
