#!/usr/bin/env python

# This module contains some utilities to simplify the computation of the angles between vectors.

import math
from numpy import *


# Versors of the axes
ux = array([1, 0, 0])
uy = array([0, 1, 0])
uz = array([0, 0, 1])


# Returns the angle between two vectors,
# by inverting the formula of the dot product.
def vectorsAngle(v1, v2):

    nv1 = linalg.norm(v1);
    nv2 = linalg.norm(v2);
    p = dot(v1, v2)

    return math.acos( p / (nv1 * nv2) )
    

# Return the projection of a vector on the given plane,
# by setting the other component to zero.

def YZ(v):  # projection on YZ, remove X
    w = v
    w[0] = 0
    return w

def XZ(v):  # projection on XZ, remove Y
    w = v
    w[1] = 0
    return w
  
def XY(v):  # projection on XY, remove Z
    w = v
    w[2] = 0
    return w

