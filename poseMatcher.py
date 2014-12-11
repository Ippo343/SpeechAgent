#!/usr/bin/env python

# This node listens for the angles of the body and tries to match them to the poses stored in the database.
# The result of the matching is then published as a PoseNotification msg for other nodes to use

# ROS connection
import rospy
from std_msgs.msg import Header
from pose_detection.msg import *
from numpy import *

# All the importing of the database is in this module
from libParsePose import *

# Global names for nodes and topics
from globalNames import *


####################### POSE MATCHING


# Default matching tolerance
defTol = 0.12

# Global names for the database and the publisher
poseDB = {}
publisher = 0

# Special value to indicate the lack of a matching pose
noMatch   = ()

# Count the notifications sent
msgCount = 0


# Tries to find a match for a pose in the database
# If the distance between the poses is greater than tol, a noMatch is returned
def findMatch(ang, DB):

    # Distance of each pose from the set of angles.
    # Each pose is actually a dictionary and the angles are contained in pose['angles'].
    # The angles to match are a fixed parameter (ang) so that it can be mapped on lists.
    def dist(pose):
        return linalg.norm(ang - pose['angles'])

    # The best matching pose is the one that minimizes the distance.
    # Here min works basically as argmin for the function "dist" defined just above.
    bestMatch = min(DB.values(), key=dist)
    bestDist  = linalg.norm(ang - bestMatch['angles'])
    
    # If the distance is not within tolerance, return a noMatch.
    # Each pose might have its own matching tolerance, so use the correct one.
    if bestDist < (defTol if bestMatch['tol'] < 0 else bestMatch['tol']):
        return bestMatch
    else:
        return noMatch
    


# Converts a BodyAngles message to a vector of angles.
# IMPORTANT: make sure this has the same order of the BodyAngles.msg file!
def fromBodyAngles(msg):

    LAF = msg.LeftArmFrontal
    LAT = msg.LeftArmTransverse
    LEA = msg.LeftElbow
    
    RAF = msg.RightArmFrontal
    RAT = msg.RightArmTransverse
    REA = msg.RightElbow
    
    HAF = msg.HeadFrontal
    
    return array([LAF, LAT, LEA, RAF, RAT, REA, HAF])



# Each time an angle message is received, look for a match.
# If it is found, publish a notification.
def angleCallback(msg):

    global msgCount    # needed to reassign it

    # Get the angles and match them to the database
    angles = fromBodyAngles(msg) 
    match = findMatch(angles, poseDB)
    
    # Send the notification    
    msgCount += 1
    
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.seq = msgCount
    
    if match == noMatch:
        notification = PoseNotification(header, 'NONE', '', '')
        print 'No matching pose.'
    else:
        notification = PoseNotification(header, match['name'], match['phrase'], match['command'])
        print 'Matched to %s; phrase: < %s >, command: < %s >' % (match['name'], match['phrase'], match['command'])
    
    # Publish it on the topic
    publisher.publish(notification)




# Main: load the database, start listening
if __name__ == '__main__':

    poseDB = importDB()    
    print "Database loaded, now listening for the angles."
    
    publisher = rospy.Publisher(notificationTopic, PoseNotification)
        
    rospy.init_node(matcherNode, anonymous=True)
    rospy.Subscriber(anglesTopic, BodyAngles, angleCallback)
    rospy.spin()    # keeps listening forever
