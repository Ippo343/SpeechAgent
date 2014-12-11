#!/usr/bin/env python

# This node reads the skeleton transforms sent by the kinect and publishes the body angles.
# Additionally, it can also dump all the angles that are recorded to a file.

# Needed to interface to ROS and the kinect
import roslib
import rospy
import tf

# To compute angles and stuff
import math
from numpy import *
from vectorUtils import *

# Needed to deal with the dump files
import os
import os.path
import sys
import datetime

# BodyAngles message (see the file BodyAngles.msg)
from pose_detection.msg import BodyAngles

from globalNames import *


###################### ROS PARAMETERS

loopRate  = 60;              # Hz

# Reference frames' names
rootRef = '/torso_1'        # root reference frame

LS_rf = '/left_shoulder_1'  # left arm frames
LE_rf = '/left_elbow_1'
LH_rf = '/left_hand_1'

RS_rf = '/right_shoulder_1' # right arm frames
RE_rf = '/right_elbow_1'
RH_rf = '/right_hand_1'

He_rf = '/head_1'           # head and neck frames
Ne_rf = '/neck_1'



###################### UTILITIES


# Use the command line argument "-d" to enable the dumping to file
DUMP = False

# Ensures that all the parents for the directory exist
def ensure_dir(f):
    d = os.path.dirname(f)
    if not os.path.exists(d):
        os.makedirs(d)
            
# Format the angles for the files
def anglesToLine(angles):
    f = lambda x: "{:1.5f}".format(x)
    return (", ".join(map(f, list(angles))))
    


###################### COMPUTATION OF THE ANGLES


# Compute the elbow angle given the coordinates of the shoulder, elbow and hand points
# The elbow angle is measured between the arm and the forearm
def elbowAngle(shoulder, elbow, hand):
    v1 = elbow - shoulder
    v2 = hand - elbow
    # Return the complement
    # (Otherwise you get the angle between the *direction* of the arm and the forearm)
    return (math.pi - vectorsAngle(v1, v2))

    

# Computes the body angles from a published set of transforms
# (Core functionality of this module)
# The angles are returned as a multiple of pi
def computeAngles(listener):

    #### First part: find the points in the reference frame, convert to vectors      
    
    # Left arm
    (Tls, _) = listener.lookupTransform(rootRef, LS_rf, rospy.Time(0));     Tls = array(Tls)
    (Tle, _) = listener.lookupTransform(rootRef, LE_rf, rospy.Time(0));     Tle = array(Tle)
    (Tlh, _) = listener.lookupTransform(rootRef, LH_rf, rospy.Time(0));     Tlh = array(Tlh)
    vLA = Tle - Tls
            
    # Right arm
    (Trs, _) = listener.lookupTransform(rootRef, RS_rf, rospy.Time(0));     Trs = array(Trs)
    (Tre, _) = listener.lookupTransform(rootRef, RE_rf, rospy.Time(0));     Tre = array(Tre)
    (Trh, _) = listener.lookupTransform(rootRef, RH_rf, rospy.Time(0));     Trh = array(Trh)
    vRA = Tre - Trs
                
    # Neck and head
    (The, _) = listener.lookupTransform(rootRef, He_rf, rospy.Time(0));     The = array(The)
    (Tne, _) = listener.lookupTransform(rootRef, Ne_rf, rospy.Time(0));     Tne = array(Tne)
    vNH = The - Tne
                
    #### Second part: actual computation
    # Frontal plane angles are measured from the vertical direction towards the link
    # Transverse plane angles are measured from the horizontal axis directed TOWARDS the kinect
    # Only exception: the neck angle is measured from the left horizontal direction
    
    # Angles of the LEFT arm
    LAF = vectorsAngle(XY(vLA),  uy) / math.pi    # with frontal plane
    LAT = vectorsAngle(XZ(vLA), -uz) / math.pi    # with transverse plane
    LEA = elbowAngle(Tls, Tle, Tlh)  / math.pi    # elbow angle
    
    # Angles of the RIGHT arm
    RAF = vectorsAngle(XY(vRA),  uy) / math.pi   # with frontal plane
    RAT = vectorsAngle(XZ(vRA), -uz) / math.pi   # with transverse plane
    REA = elbowAngle(Trs, Tre, Trh)  / math.pi   # elbow angle
    
    # Angles of the neck / head link
    HAF = vectorsAngle(XY(vNH),  ux) / math.pi   # with frontal plane
    

    # The returned angles are all normalized over pi
    result = (LAF, LAT, LEA, RAF, RAT, REA, HAF)
    
    if DUMP: # dump requested, write the angles to the file
        dumpFile.write(anglesToLine(result) + "\n")
    
    return result
    



###################### MAIN LOOP


if __name__ == '__main__':

    # Look for the dump option
    DUMP = any(map(lambda s: (s == '-d'), rospy.myargv(argv=sys.argv)))
 
    # Initialize variables needed for the dump and open the file
    if DUMP == True:

        # The file is saved to a text file:
        #   - placed in ~/Desktop/DUMP
        #   - called with the datetime the program was started
        #   - in csv format, with the first line as header
        fileName = str(datetime.datetime.now()) + '.txt'
        dumpDir  = os.path.expanduser('~') + '/Desktop/DUMP/'; ensure_dir(dumpDir)
        dumpPath = dumpDir + fileName    
        dumpFile = open(dumpPath, 'w')   
        dumpFile.write("      ".join(["LAF", "LAT", "LEA", "RAF", "RAT", "REA", "HAF", "\n"]))
        
        print "Dumping all the output to %s" % dumpPath
        
        

    # Initialize the node and the message publisher
    print 'Initializing the node...'
    rospy.init_node(anglesNode)
    publisher = rospy.Publisher(anglesTopic, BodyAngles)

    # Get the transform listener
    listener = tf.TransformListener()
    print 'Now listening for the skeleton transforms (%d Hz).' % loopRate

    rateChecker = rospy.Rate(loopRate)
    while not rospy.is_shutdown():
        try:
        
            # Compute the angles
            (LAF, LAT, LEA, RAF, RAT, REA, HAF) = computeAngles(listener)
            
            # Convert to message and publish
            message = BodyAngles(LAF, LAT, LEA, RAF, RAT, REA, HAF)
            publisher.publish(message)
            print "PUBLISHED: %s" % anglesToLine( (LAF, LAT, LEA, RAF, RAT, REA, HAF) )
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # if some exception happens during the lookup, just do nothing
            () # (tipically because no transform is being broadcast)

        rateChecker.sleep() # force the execution rate

