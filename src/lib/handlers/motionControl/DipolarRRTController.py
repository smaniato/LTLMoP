#!/usr/bin/env python
"""
==================================================
DipolarController.py - Dipolar Motion Controller
==================================================

This class uses a dipolar field along with an RRT to generate paths for the
robot as well as the linear and angular velocities to drive them.
"""

from __future__ import division

from Polygon import Polygon
import numpy as np
from _DipolarRRT import RRTMap, RRTPlotter, RRTRobot, DipolarRRT, DipolarController, diffAngles


class motionControlHandler:
    def __init__(self, proj, shared_data, robotType, nodeDistInter, dipolarGain):
        """
        An RRT with dipoles for connecting nodes.
        
        robotType (int): The robot shape to be used. Circle with radius .1 is 0. (default=0)
        nodeDistInter (float): The max euclidean distance between nodes. (default=1)
        dipolarGain (float): The k1 gain for the dipolar closed loop controller. (default=0.5)
        """
        # Settings
        self.DEBUG = False           # Print statements for debugging
        self.DEBUGER = False        # If using a debugger. Matplotlib workaround
        self.PLOT_REG = True       # Plot the current and next region
        self.PLOT_TREE = True      # Plot the RRT live
        self.PLOT_TREE_FAIL = True # Plot the RRT if it fails to find a path
        self.PLOT_PATH = True       # Plot path generated
        self.closeEnoughDist = 1   # The max distance from waypoint
        self.closeEnoughAng = .5   # The max angle difference from pose
        
        # Get references to handlers we'll need to communicate with
        self.drive_handler = proj.h_instance['drive']
        self.pose_handler = proj.h_instance['pose']
        
        # Get information about regions
        self.rfi = proj.rfi
        self.coordmap_map2lab = proj.coordmap_map2lab
        
        # Plotter
        self.plotter = RRTPlotter(invertY=True)
        self.plotter.ion()                       # Turn on interactive mode
        
        # Dipolar controller
        self.dipController = DipolarController(k1=dipolarGain)
        self.prevPose = self.pose_handler.getPose()
        self.dT = 1/20.0    # The time elapsed since the call to controller
        
        # RRT Variables
        self.robot = self.getRobot(robotType)
        self.nodeDistInter = nodeDistInter
        self.path = None
        self.nextWaypointIndex = None
        self.storedNextReg = None
        self.rrt = DipolarRRT(None, self.robot, plotter=self.plotter, 
                              dipController=self.dipController)
        self.rrt.setCloseEnoughVals(self.closeEnoughDist, self.closeEnoughAng)
        self.rrt.setConnectDist(nodeDistInter)

    def gotoRegion(self, current_reg, next_reg, last=False):
        """ Returns ``True`` if we've reached the next region. Uses the generated path
        and dipoles to go to the next waypoint. If the desired region changes or no path
        currently excists, it will create one using the RRT
        """

        if current_reg == next_reg and not last:
            # No need to move!
            self.drive_handler.setVelocity(0, 0)  # So let's stop
            return True  # Already there
        
        # Check if a new path needs to be calculated
        if self.path is None or self.storedNextReg != next_reg:
            self.drive_handler.setVelocity(0, 0)
            self.findAndSetPath(current_reg, next_reg)
            self.storedNextReg = next_reg
            
        # Find our current configuration
        pose = self.pose_handler.getPose()
            
        # Check if reached waypoint
        while self.closeEnough(pose, self.path[self.nextWaypointIndex]):
            self.nextWaypointIndex += 1
            
            # Check end of path
            if self.nextWaypointIndex >= len(self.path):
                self.path = None
                self.drive_handler.setVelocity(0, 0)
                return True  # Already there

        # Calculate the linear and angular velocities
        nextWaypoint = self.path[self.nextWaypointIndex]
        v, w = self.dipController.getControlls(pose, nextWaypoint, 
                                               self.prevPose, self.dT)
        self.drive_handler.setVelocity(v, w)
        self.prevPose = pose  
        
        if self.DEBUG:
            print "-----------------"
            print "Current pose:", pose
            print "Next waypoint:", nextWaypoint
            print "V:", v, "    W:", w

        return False  
    
    def findAndSetPath(self, current_reg, next_reg):
        """ Calculate the path to go from current_reg to next_reg. """
        
        # Current and next regions
        currRegBoundary, currRegObstList = self.getRegionPolygons(current_reg)
        nextRegBoundary, nextRegObstList = self.getRegionPolygons(next_reg)
        allObstacles = currRegObstList + nextRegObstList
        
        currRegPoly = currRegBoundary
        for obst in currRegObstList:
            currRegPoly -= obst
        
        nextRegPoly = nextRegBoundary
        for obst in nextRegObstList:
            nextRegPoly -= obst
        
        # Prepare RRT
        rrtMap = RRTMap(currRegPoly + nextRegPoly)
        
        pose = self.pose_handler.getPose()
        
        self.rrt.updateMap(rrtMap)
        self.rrt.DEBUGER = self.DEBUGER
        self.rrt.PLOT_TREE = self.PLOT_TREE
        self.rrt.PLOT_TREE_FAIL = self.PLOT_TREE_FAIL
        self.rrt.connectDist = self.nodeDistInter
        
        # Goal poses
        goalPoseList = self.getGoalPoses(current_reg, next_reg, nextRegPoly)    
        if len(goalPoseList) == 0:
            raise Exception("Error: DipolarRRTController - No goal pose found.")
        
        if self.DEBUG:
            print "GoalPose:", goalPoseList
        
        # Start plotting and generating the path
        self.plotter.clearPlot()
        
        if self.PLOT_REG:
#             self.plotter.drawMap(rrtMap)
            self.plotter.drawPolygon(currRegBoundary, color='r', width=3)
            self.plotter.drawPolygon(nextRegBoundary, color='g', width=3)
            for obst in allObstacles:
                self.plotter.drawPolygon(obst, color='k', width=3)
            for goalPose in goalPoseList:
                self.plotter.drawStartAndGoalRobotShape(pose, goalPose, self.robot)
            
        if self.DEBUG:
            print "Getting RRTPath"
            
        rrtPath = self.rrt.getRRTDipoleControlPath(pose, goalPoseList)
    
        if self.PLOT_PATH:
            pathT = self.rrt.get2DPathRepresent(rrtPath)
            self.plotter.drawDipolePath2D(pathT, color='g', width=3)
            
        if self.DEBUG:
            print "Getting Short Path"
            
        allGoalNodes = self.rrt.dipolesToNodes(goalPoseList)
#         shortPath = self.rrt.getShortcutPathDipole(rrtPath, additionalGoals=allGoalNodes)
        shortPath = self.rrt.getThetaStarPath(rrtPath, additionalGoals=allGoalNodes)
        
        if self.PLOT_PATH:
            pathT = self.rrt.get2DPathRepresent(shortPath)
            self.plotter.drawDipolePath2D(pathT, color='r', width=3)
            
        # Update instance fields
        self.path = self.rrt.nodesToDipoles(shortPath)    # Convert to 3D vector
        self.nextWaypointIndex = 0
        self.storedNextReg = next_reg
        
    def getRegionPolygons(self, targetReg):
        """ Returns a polygon representing the region boundary and a list
        of polygons representing the obstacles or holes in the region. 
        """
        # Get boundary polygon
        region = self.rfi.regions[targetReg]
        boundaryPoints = [x for x in region.getPoints()]
        boundaryPoints = map(self.coordmap_map2lab, boundaryPoints)
        regionPoly = Polygon(boundaryPoints)
        
        # Remove holes
        obstacleList = []
        for holeId in range(len(region.holeList)):
            pointsT = [x for x in region.getPoints(hole_id=holeId)]
            pointsT = map(self.coordmap_map2lab, pointsT)
            obstacleList.append(Polygon(pointsT))
        
        return regionPoly, obstacleList
    
    def getGoalPoses(self, currRegion, nextRegion, nextRegPoly):        
        """ Returns a list of numpy 1 x 3 numpy arrays with possible goal poses
          
        For each transition face between the two regions: Find the center point 
        and place goal points such that they extend from the center points in the 
        direction of the normal to the transition face. The normal should point into
        the next region and the points will be a distance slightly larger than 
        robot.backLen away from the center point from which they extend. Goal poses
        will be the goal points with the direction of the face's normal as the third 
        element.
        Take a deep breath and read that over.
        """
#         distIntoPoly = self.robot.backLen * 1.1 + self.closeEnoughDist + 20
        distIntoPoly = self.robot.backLen * 1.1 + self.closeEnoughDist
    
        robotCopy = self.robot.copy()
        
        goalPoses = []
        for transFace in self.rfi.transitions[currRegion][nextRegion]:
            transFacePoints = [x for x in transFace]
            transFacePoints = np.array(map(self.coordmap_map2lab, transFacePoints))
            
            center = (transFacePoints[0,:] + transFacePoints[1,:]) / 2
            faceDir = transFacePoints[1,:] - transFacePoints[0,:]
            faceNorm = np.array([faceDir[1], -faceDir[0]])
            faceNorm = faceNorm/np.linalg.norm(faceNorm) * distIntoPoly
            
            # Check for correct direction face norm
            pointT = center + faceNorm
            if not nextRegPoly.isInside(pointT[0], pointT[1]):
                faceNorm *= -1
            
            # Direction and point for the goalPose
            dirT = np.arctan2(faceNorm[1], faceNorm[0])
            pointT = center + faceNorm
            poseT = np.array([pointT[0], pointT[1], dirT])
            robotCopy.moveRobotTo(poseT)
            if nextRegPoly.covers(robotCopy.shape):
                goalPoses.append(poseT)
                
#             print "TPoints:", transFacePoints
#             print "C:", center
#             print "Norm:", faceNorm
#             print "PoseT:", poseT
            
        return goalPoses    
        
    def closeEnough(self, pose1, pose2):
        """ Returns true if pose1 and pose2 are within a threshold distance
        and angle difference.
        """       
        dist = np.linalg.norm(pose1[:2] - pose2[:2])
        angDiff = abs(diffAngles(pose1[2], pose2[2]))
        
        if dist < self.closeEnoughDist and angDiff < self.closeEnoughAng:
            return True
        else:
            return False
        
    def getRobot(self, robotType):
        """ Returns a predefined RRTRobot """
        # Simulated Circular robot (For when lab to map matrix is the identity)
        if robotType == 0:
            return RRTRobot.circularRobot([0,0,0], 10)
        else:
            msg = "ERROR: DipolarRRTController - Undefined robot type."
            raise Exception(msg)
        
        
        
    
    
    