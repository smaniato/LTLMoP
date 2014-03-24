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
    
    def __init__(self, robot, polygons, thetaConstraints=None, areGoals=None):
        """ thetaConstraints is a list of constraints where each
        constraint has the form (angle, maxOffset). 
        
        :param robot: An RRTRobot to use for collision checks
        :param polygons: A list of polygons representing regions.
        :param thetaConstraints: A list of (angle, maxOffsets) for each region.
            None if no constraints.
        :param areGoals: A list of booleans marking regions as goals or not.
            None if no region is a goal.
        """
        numPoly = len(polygons)
        
        # Make personal copies of the parameters
        self.robot = robot.copy()
        polygons = [Polygon.Polygon(p) for p in polygons]
        if thetaConstraints is not None:
            thetaConstraints = list(thetaConstraints)
        else:
            thetaConstraints = [(0, pi)]*numPoly
        if areGoals is not None:
            areGoals = list(areGoals)
        else:
            areGoals = [False]*numPoly
            
        # To use for copy()
        self.polygons = polygons
        self.thetaConstraints = thetaConstraints
        self.areGoals = areGoals

        # Regions are (polygon, constraint) pairs
        self.regions = zip(polygons, thetaConstraints)
        self.goalRegions = [self.regions[i] for i in range(numPoly) if areGoals[i]]
        
        # Poly sums are cumulative free spaces
        goalPolygons = (polygons[i] for i in range(numPoly) if areGoals[i])
        self.polySum = reduce(lambda x, y: x + y, polygons)
        self.goalPolySum = reduce(lambda x, y: x + y, goalPolygons)
        
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
        """ Samples a pose inside of regions, but does not perform
        collision checks. 
        """
        x, y = polySum.sample(random)
        
        # Generate random angles for all regions that x, y is in 
        randAngles = []
        for poly, (theta, off) in regions:
                if poly.isInside(x, y):
                    randAngles.append(random() * 2 * off - off + theta)
                    
        # Choose one of the angles
        if len(randAngles) > 0:
            randAng = sample(randAngles, 1)[0]
        else:
            randAng = 0
        
        return (x, y, randAng)
    
    def samplePose(self):
        """ Sample a collision free pose for robot within any region.
        """
        MAX_ITER = 1000      # Maximum number of attempts
        
        robot = self.robot
        for _ in range(MAX_ITER):
            pose = self.sample(self.polySum, self.regions)
            robot.moveTo(pose)
            if self.isCollisionFree(robot):
                return pose
            
        raise Exception("Could not sample a pose...")
        
    def sampleGoal(self):
        MAX_ITER = 1000      # Maximum number of attempts
        
        robot = self.robot
        for _ in range(MAX_ITER):
            pose = self.sample(self.goalPolySum, self.goalRegions)
            robot.moveTo(pose)
            if self.isInGoal(robot):
                return pose
            
        raise Exception("Could not sample goal...")
    
    def addRobotBuffer(self, robot):
        """ Use if robot may start sampling and is not completely
        within any of the regions. Increases the size of poly sums by
        a scaled robot shape. 
        Note: Polygons are changed. Consider making a copy of the map
        to restore back to the original.
        """
        scale = 1.3
        xT, yT, _ = robot.pose
        robotP = Polygon.Polygon(robot.shape)
        robotP.scale(scale, scale, xT, yT)
        
        if self.polySum.overlaps(robotP):
            self.polySum += robotP
        if self.goalPolySum.overlaps(robotP):
            self.goalPolySum += robotP
            
    def copy(self):
        return RRTMapConst(self.robot, self.polygons, self.thetaConstraints, 
                           self.areGoals)
            
class RRTRobot:
    
    # TODO: REMOVE RADIUS AS INPUT (CALCULATE IN CODE IF NEEDED)
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

