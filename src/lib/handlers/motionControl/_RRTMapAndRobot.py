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


class RRTMap:
    
    def __init__(self, boundary, allObstacles=None):
        ''' An object with the outline of the map, a list of obstacles,
        and the poses that the robot starts and ends on.
        
        :param boundary: shape
        :param allObstacles: list of polygons
        :param startPose: numpy 3D array
        :param endPose: numpy 3D array
        '''
        self.boundary = boundary
        if allObstacles == None:
            self.allObstacles = []
        else:
            self.allObstacles = allObstacles
            
        self.cFree = Polygon.Polygon(boundary)
        for obst in self.allObstacles:
            self.cFree -= obst
            
            
class RRTRobot:
    
    def __init__(self, pose, outline, radius=None):
        """ An object that represents the robot location and outline.
        
        :param pose: numpy 3D array
        :param outline: Polygon describing the space occupied by the robot at 
                        the current pose 
        :param radius: radius of a circle encompasing the robot and centered at
                        pose. Only used for 2D RRT
        """
        self.pose = np.array(pose)
        self.shape = outline
        self.radius = radius
        
    @classmethod
    def circularRobot(cls, pose, radius=None):
        """ Create a circular representation of a robot.
        
        :param pose: numpy 3D array
        :param radius: radius of the robot
        """
        center = (pose[0], pose[1])
        tempPoly = pShapes.Circle(radius, center)
        
        return cls(pose, tempPoly, radius)
        
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
        return RRTRobot(np.array(self.pose), Polygon.Polygon(self.shape))


def diffAngles(angle1, angle2):
    """ Returns difference between -pi and pi. Relative to angle2.
    """
    return (angle1 - angle2 + np.pi)%(2*np.pi) - np.pi

