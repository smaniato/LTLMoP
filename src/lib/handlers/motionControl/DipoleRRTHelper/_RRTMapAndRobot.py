#!/usr/bin/env python
"""
==================================================
RRTMapAndRobot
==================================================

This class holds representations for maps and robot
polygons that are used in DipolarRRT.
"""

from math import pi
from random import random, sample

import Polygon

import Polygon.Shapes as pShapes
import numpy as np


Polygon.setTolerance(0.01)


class RRTMap:
    
    def __init__(self, robot, polygons, thetaConstraints=None):
        """ Creates an object to represent a map. 
        
        :param robot: An RRTRobot object to use for collision checks
        :param polygons: A list of polygons to represent regions
        :param thetaConstraints: A list of (angle, maxOffset) for each region
            (None if no constraints)
        """
        numPoly = len(polygons)
        
        # Make personal copies of the parameters
        self.robot = robot.copy()
        polygons = [Polygon.Polygon(p) for p in polygons]
        if thetaConstraints is not None:
            thetaConstraints = list(thetaConstraints)
        else:
            thetaConstraints = [(0, pi)]*numPoly

        # Regions are (polygon, constraint) pairs
        # Poly sums are cumulative free spaces
        self.regions = zip(polygons, thetaConstraints)
        self.polySum = reduce(lambda x, y: x + y, polygons)
        
    def meetsRegionConstraints(self, robot):
        """ Checks if the given robot fits within polySum and follows the 
        orientational constraint of at least one of the regions that its 
        pose falls into. 
        """
        x, y, thetaR = robot.pose
        if self.polySum.covers(robot.shape):
            for poly, (theta, off) in self.regions:
                if (abs(diffAngles(theta, thetaR)) < off and 
                    poly.isInside(x, y)):
                    return True
        return False
    
    def isValidPose(self, pose):
        self.robot.moveTo(pose)
        return self.meetsRegionConstraints(self.robot)
    
    def sample(self):
        """ Samples a pose inside of regions, but do not perform
        collision checks. 
        """
        x, y = self.polySum.sample(random)
        
        # Generate random angles for all regions that x, y is in 
        randAngles = []
        for poly, (theta, off) in self.regions:
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
            pose = self.sample()
            robot.moveTo(pose)
            if self.meetsRegionConstraints(robot):
                return pose
            
        raise Exception("Could not sample a pose...")
    
    def addRobotBuffer(self, robot):
        """ Use if robot may start sampling and is not completely
        within any of the regions. Increases the size of poly sums by
        a scaled robot shape. 
        Note: Robot pose must be within part of the map.
        Note: Polygons are changed. Consider making a copy of the map
        to restore back to the original.
        """
        scale = 1.3
        xT, yT, _ = robot.pose
        if self.polySum.isInside(xT, yT):
            robotP = Polygon.Polygon(robot.shape)
            robotP.scale(scale, scale, xT, yT)
            self.polySum += robotP
            
    def copy(self):
        poly, const = zip(*self.regions)
        return RRTMap(self.robot, poly, const)
            
class RRTRobot:
    
    # TODO: REMOVE RADIUS AS INPUT (CALCULATE IN CODE IF NEEDED)
    def __init__(self, pose, outline):
        """ An object that represents the robot location and outline.
        
        :param pose: The pose of the robot. Must match the polygon passed in.
        :param outline: Polygon describing the space occupied by the robot at 
                        the current pose
        """
        self.pose = np.array(pose, dtype=float)
        self.shape = Polygon.Polygon(outline)
        
    @classmethod
    def circularRobot(cls, pose, radius=None):
        """ Create a circular representation of a robot.
        """
        center = pose[:2]
        tempPoly = pShapes.Circle(radius, center)
        
        return cls(pose, tempPoly)
        
    def moveTo(self, newPose):
        """ Updates the robot pose and outline to match the new pose
        
        :param newPose: The pose to move the robot to
        """
        dirV = newPose[:2] - self.pose[:2]
        angleDiff = diffAngles(newPose[2], self.pose[2])
        self.shape.shift(dirV[0], dirV[1])
        self.pose[:3] = newPose
        self.shape.rotate(angleDiff, newPose[0], newPose[1])
    moveRobotTo = moveTo        # Will be removed (deprecated)
    
    def moveToPosition(self, newPosition):
        """ Updates the robot pose and outline to match the new pose
        
        :param newPosition: The position to move to
        """
        dirV = newPosition - self.pose[:2]
        self.shape.shift(dirV[0], dirV[1])
        self.pose[:2] = newPosition
    moveRobotToPosition = moveToPosition
        
    def copy(self):
        return RRTRobot(self.pose, self.shape)


def diffAngles(angle1, angle2):
    """ Returns difference between -pi and pi. Relative to angle2.
    """
    return (angle1 - angle2 + pi)%(2*pi) - pi

