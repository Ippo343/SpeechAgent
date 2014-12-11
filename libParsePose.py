#!/usr/bin/env python

# Functions to import the pose database


# Needed to deal with the path of the DB
import os
import os.path

# The pose files as saved in text files that MUST contain valid python code.
# This is very bad and very dangerous, but there's not time to use XML.
# ast.literal_eval will only evaluate code that parses to valid python datatypes:
# This at least ensures that only variables are given to eval(), and we get an exception if it's malformed
from ast import literal_eval

# Needed for the array type (vectors)
from numpy import *


# Path of the folder that contains the .pose files,
# located in ~/poses
DBdir = os.path.expanduser('~') + '/poses'



# Enters the database folder and filters the .pose files
def findPoseFiles():

    # Returns true if the file has the extension ".pose"
    def isPoseFile(f):
        (_, ext) = os.path.splitext(f)
        return True if (os.path.isfile(os.path.join(DBdir,f)) and ext == '.pose') else False
    
    # List the files in the DB dir, and filter the ones with the valid extension          
    poseFiles = [ os.path.join(DBdir,f) for f in os.listdir(DBdir) if isPoseFile(f) ]
    
    return poseFiles




# Parse a pose file and return a dictionary
# The .pose files must contain valid python code to produce a dictionary,
# with the following fields:
#   -   name: a string with the name of the pose
#   - angles: a list (or a tuple) with the value of the angles. Make sure they are in the correct order as defined in BodyAngles.msg!
#   - phrase: the phrase that the agent must say in response to the pose
#   -    tol: the tolerance for the match of this pose. If negative, the default one will be used
def poseFromFile(path):

    # Try to parse the file as a dictionary
    f = open(path, 'r')
    p = literal_eval(f.read())
    
    # At this point, either we raised an exception or d is the pose
    # TODO: check also that the required fields are in the dictionary
    
    # Check that the file name matches the pose name, and give a warning if it doesn't.
    # The name specified in the file will always win over the file name.
    (_, tail) = os.path.split(path)
    (name, _) = os.path.splitext(tail)
    if not (name == p['name']):
        print "Warning: file name and pose name don't match."
    
    # Convert to numpy array for later computation
    assert len(p['angles']) == 7    
    p['angles'] = array(p['angles'])
   
    return p
    
    


# Reads all the pose files to produce the database.
# The database is a dictionary of poses:
# since every pose is also a dictionary, then it's a dictionary of dictionaries.
# The keys for the database are the names of the poses, which are equal to the ones saved into the poses themselves.
def importDB():

    print "Importing the poses database from folder %s" % DBdir

    DB = {}  
    poseFiles = findPoseFiles()
    
    print "Found %i .pose files." % len(poseFiles)
    
    for f in poseFiles:
        
        print "\tParsing file %s" % f
        
        p = poseFromFile(f)        
        DB[ p['name'] ] = p
       
    return DB
