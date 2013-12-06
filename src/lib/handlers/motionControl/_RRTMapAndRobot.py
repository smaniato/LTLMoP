#!/usr/bin/env python
"""
==================================================
RRTMapAndRobot
==================================================

This class holds representations for maps and robot
polygons that are used in DipolarRRT.
"""

import Polygon

import Polygon.Shapes as pShapes
import numpy as np
import cPickle as pickle
from _DipolarRRT import diffAngles 


class RRTMap:
    
    def __init__(self, mapArea, allObstacles=None):
        ''' An object with the outline of the map, a list of obstacles,
        and the poses that the robot starts and ends on.
        
        :param mapArea: Polygon representative of the area
        :param allObstacles: list of polygons
        '''
        self.boundary = mapArea
        if allObstacles == None:
            self.allObstacles = []
        else:
            self.allObstacles = allObstacles
            
        self.cFree = Polygon.Polygon(mapArea)
        for obst in self.allObstacles:
            self.cFree -= obst
            
class RRTRobot:
    
    def __init__(self, pose, outline, backLen, radius=None):
        """ An object that represents the robot location and outline.
        
        :param pose: numpy 3D array
        :param outline: Polygon describing the space occupied by the robot at 
                        the current pose 
        :param backLen: The length of the robot from it center to the furthest most
                        back point. Used for calculating region enter distance.
        :param radius: radius of a circle encompasing the robot and centered at
                        pose. Only used for 2D RRT
        """
        self.pose = np.array(pose)
        self.shape = outline
        self.backLen = backLen
        self.radius = radius
        
    @classmethod
    def circularRobot(cls, pose, radius=None):
        """ Create a circular representation of a robot.
        
        :param pose: numpy 3D array
        :param radius: radius of the robot
        """
        center = (pose[0], pose[1])
        tempPoly = pShapes.Circle(radius, center)
        
        return cls(pose, tempPoly, radius, radius=radius)
        
    def moveRobotTo(self, newPose):
        """ Updates the robot pose and outline to match the new pose
        
        :param newPose: numpy 3D array
        """
        # TODO: DIR V DOES NOT NEED BE A VECTOR SPLIT IT UP
        dirV = newPose[:2] - self.pose[:2]
        angleDiff = diffAngles(newPose[2], self.pose[2])
        self.shape.shift(dirV[0], dirV[1])
        # TODO: DONT MAKE A NEW ARRAY JUST UPDATE VALUES
        self.pose = np.array(newPose)
        self.shape.rotate(angleDiff, newPose[0], newPose[1])
        
    def copy(self):
        return RRTRobot(np.array(self.pose), Polygon.Polygon(self.shape),
                         self.backLen)


