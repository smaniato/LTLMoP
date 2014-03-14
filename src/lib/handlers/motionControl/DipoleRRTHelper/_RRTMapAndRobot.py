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
from math import pi
from random import random, sample


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
            
class RRTMapConst:
    
    def __init__(self, polygons, thetaConstraints=None, areGoals=None):
        """ thetaConstraints is a list of constraints where each
        constraint has the form (angle, maxOffset)
        """
        numPoly = len(polygons)
        
        if thetaConstraints is None:
            thetaConstraints = [(0, pi)]*numPoly
            
#         print "Remove this"
#         thetaConstraints = [(0, pi)]*len(polygons)

        self.regions = zip(polygons, thetaConstraints)
        self.polySum = reduce(lambda x, y: x+y, polygons)
        
        if areGoals is not None:
            goalPolygons = [polygons[i] for i in range(len(polygons))
                            if areGoals[i]]
            goalConst = [thetaConstraints[i] for i in range(len(polygons))
                            if areGoals[i]]
            self.goalRegions = zip(goalPolygons, goalConst)
            self.goalPolySum = reduce(lambda x, y: x+y, goalPolygons)
        else: 
            self.goalRegions = None
            self.goalPolySum = None
        
    def meetsRegionConstraints(self, robot, polySum, regions):
        x, y, thetaR = robot.pose
        if polySum.covers(robot.shape):
            for poly, (theta, off) in regions:
                if (abs(diffAngles(theta, thetaR)) < off and 
                    poly.isInside(x, y)):
                    return True
        return False
    
    def isCollisionFree(self, robot):
        isIn = self.meetsRegionConstraints
        return isIn(robot, self.polySum, self.regions)
    
    def isInGoal(self, robot):
        isIn = self.meetsRegionConstraints
        return isIn(robot, self.goalPolySum, self.goalRegions)
    
    def sample(self, polySum, regions):
        x, y = polySum.sample(random)
        randAngles = []
        for poly, (theta, off) in regions:
                if poly.isInside(x, y):
                    randAngles.append(random() * 2 * off - off + theta)
        randAng = sample(randAngles, 1)[0]
        
        return (x, y, randAng)
    
    def samplePose(self, robot):
        MAX_ITER = 500      # Maximum number of attempts
        
        for i in range(MAX_ITER):
            pose = self.sample(self.polySum, self.regions)
            robot.moveTo(pose)
            if self.isCollisionFree(robot):
#                 print "RegIter: ", i
                return pose
            
        raise Exception("Could not sample a pose...")
        
    def sampleGoal(self, robot):
        MAX_ITER = 500      # Maximum number of attempts
        
        for i in range(MAX_ITER):
            pose = self.sample(self.goalPolySum, self.goalRegions)
            robot.moveTo(pose)
            if self.isInGoal(robot):
#                 print "GoalIter: ", i
                return pose
            
        raise Exception("Could not sample goal...")
    
    def addRobotBuffer(self, robot):
        """ Increase the space of all regions that contain the robot. Used
        to loosen constraints on initial movements due to "close enough" 
        parameters.   
        """
        scale = 1.3
        robotT = robot.copy()
        xT, yT, thetaR = robot.pose
        robotT.shape.scale(scale, scale, xT, yT)
        
        for poly, (theta, off) in self.regions:
            if (abs(diffAngles(theta, thetaR)) < off and 
                poly.isInside(xT, yT)):
                poly += robotT.shape
        
        self.polySum += robotT.shape
            
class RRTRobot:
    
    def __init__(self, pose, outline, radius):
        """ An object that represents the robot location and outline.
        
        :param pose: numpy 3D array
        :param outline: Polygon describing the space occupied by the robot at 
                        the current pose 
        :param radius: radius of a circle encompassing the robot and centered at
                        pose. Only used for 2D RRT
        """
        self.pose = np.array(pose, dtype=float)
        self.shape = Polygon.Polygon(outline)
        self.radius = radius
        
    @classmethod
    def circularRobot(cls, pose, radius=None):
        """ Create a circular representation of a robot.
        
        :param pose: numpy 3D array
        :param radius: radius of the robot
        """
        center = pose[:2]
        tempPoly = pShapes.Circle(radius, center)
        
        return cls(pose, tempPoly, radius)
        
    def moveTo(self, newPose):
        """ Updates the robot pose and outline to match the new pose
        
        :param newPose: numpy 3D array
        """
        dirV = newPose[:2] - self.pose[:2]
        angleDiff = diffAngles(newPose[2], self.pose[2])
        self.shape.shift(dirV[0], dirV[1])
        self.pose[:3] = newPose
        self.shape.rotate(angleDiff, newPose[0], newPose[1])
    moveRobotTo = moveTo        # Will be removed (deprecated)
    
    def moveToPosition(self, newPosition):
        """ Updates the robot pose and outline to match the new pose
        
        :param newPosition: numpy 2D array
        """
        dirV = newPosition - self.pose[:2]
        self.shape.shift(dirV[0], dirV[1])
        self.pose[:2] = newPosition
    moveRobotToPosition = moveToPosition
        
    def copy(self):
        return RRTRobot(self.pose, self.shape, self.radius)


def diffAngles(angle1, angle2):
    """ Returns difference between -pi and pi. Relative to angle2.
    """
    return (angle1 - angle2 + pi)%(2*pi) - pi

