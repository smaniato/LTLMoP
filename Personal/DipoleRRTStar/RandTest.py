from Queue import PriorityQueue
from random import random, randint
from time import clock
 
import cProfile as Profile
 
import Polygon
from numpy.linalg import norm
 
from _DipolarCLoopControl import DipolarController
from _PlotRRT import RRTPlotter
import Polygon.Shapes as pShapes
from _RRTMapAndRobot import RRTMap, RRTRobot
import matplotlib.pyplot as plt
import numpy as np
from math import pi


def sign(x):
    return (x > 0) - (x < 0)

def test():
    sign = lambda x: (x > 0) - (x < 0)
    [sign(x) for x in xrange(-1000,1000)]
    [np.sign(x) for x in xrange(-1000,1000)]
    
def diffAngles(angle1, angle2):
    return (angle1 - angle2 + pi)%(2*pi) - pi

def diffAngles2(angle1, angle2):
    a = angle1 - angle2
    if a < -pi: 
        a += 2*pi
    elif a > pi:
        a -= 2*pi
    return a

def diffAngles3(a1, a2):
    min((2 * pi) - abs(a1 - a2), abs(a1 - a2))

def test2():
    [diffAngles(0, x) for x in xrange(100000)]
    [diffAngles2(0, x) for x in xrange(100000)]
    [diffAngles3(0, x) for x in xrange(100000)]
    

Profile.run("test2()")