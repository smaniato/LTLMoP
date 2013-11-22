#!/usr/bin/env python
"""
==================================================
DipolarController.py - Dipolar Motion Controller
==================================================

This class uses a dipolar field along with an RRT to generate paths for the
robot as well as the linear and angular velocities to drive them.
"""
from Polygon import Polygon
from numpy import *
from DipolarRRT import RRTMap, RRTPlotter, RRTRobot, DipolarRRT, DipolarController

from time import sleep

class motionControlHandler:
    def __init__(self, proj, shared_data):
        # Settings
        robotRadius = .01
        self.DEBUGER = False        # If using a debugger. Matplotlib workaround
        self.PLOT_REG = True       # Plot the current and next region
        self.PLOT_TREE = True      # Plot the RRT live
        self.PLOT_TREE_FAIL = True # Plot the RRT if it fails to find a path
        self.PLOT_PATH = True       # Plot path generated
        
        # Get references to handlers we'll need to communicate with
        self.drive_handler = proj.h_instance['drive']
        self.pose_handler = proj.h_instance['pose']
        
        # Get information about regions
        self.rfi = proj.rfi
        self.coordmap_map2lab = proj.coordmap_map2lab
        
        # RRT Variables
        self.path = None
        self.nextWaypointIndex = None
        self.robot = RRTRobot.circularRobot([0, 0, 0], robotRadius)
        self.nextReg = None
        self.currRegPoly = None
        self.nextRegPoly = None
        
        # Dipolar controller
        self.diController = DipolarController(k1=10)
        self.prevPose = self.pose_handler.getPose()
        self.dT = 1/15.0    # The time elapsed since the call to controller
        
        # Plotter
        self.plotter = RRTPlotter()
        self.plotter.ion()                       # Turn on interactive mode

    def gotoRegion(self, current_reg, next_reg, last=False):
        """
        Returns ``True`` if we've reached the center of destination region.
        """

#         print "D-- gotoRegion()"

        # TODO: What's going on here
        if current_reg == next_reg and not last:
            # No need to move!
            self.drive_handler.setVelocity(0, 0)  # So let's stop
            sleep(.05)
            return True  # Already there
        
#         print "D-- got through part 1"
        
        # Check if a new path needs to be calculated
        if self.path is None or self.nextReg != next_reg:
            self.drive_handler.setVelocity(0, 0)
            self.findAndSetPath(current_reg, next_reg)
            self.nextReg = next_reg
            print "D-- GOT NEW PATH"
            
        # Find our current configuration
        pose = self.pose_handler.getPose()
            
#         print "D-- CHECKING WAYPOINT"
            
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
        v, w = self.diController.getControlls(pose, nextWaypoint, 
                                              self.prevPose, self.dT)
        
        print "Current pose:", pose
        print "Next waypoint:", nextWaypoint
        print "V, W", v, w

        # Pass this desired velocity on to the drive handler
        # It also gets the current heading of the robot
        self.drive_handler.setVelocity(v, w, pose[2])
        
#         # TODO: What's going on here
#         # Figure out whether we've exited the current region
#         if is_inside([pose[0], pose[1]], vertices):
#             arrived = False
#         else:
#             arrived = True

        self.prevPose = pose    
        
#         return arrived
    
    def findAndSetPath(self, current_reg, next_reg):
        """ Calculate the path to go from current_reg to next_reg. """
        
        # Current region
        currRegPoints = [x for x in self.rfi.regions[current_reg].getPoints()]
        currRegPoints = map(self.coordmap_map2lab, currRegPoints)
        currRegPoly = Polygon(currRegPoints)
        
        # Next region
        if next_reg == current_reg:
            nextRegPoints = currRegPoints
            nextRegPoly = currRegPoly
        else:
            nextRegPoints = [x for x in self.rfi.regions[next_reg].getPoints()]
            nextRegPoints = map(self.coordmap_map2lab, nextRegPoints)
            nextRegPoly = Polygon(nextRegPoints)      
            nextRegPoints = array(nextRegPoints)   
        
        # Prepare RRT
        workspace = currRegPoly + nextRegPoly
        rrtMap = RRTMap(workspace)
        
        pose = self.pose_handler.getPose()
        
        rrt = DipolarRRT(rrtMap, self.robot, self.plotter)
        rrt.DEBUGER = self.DEBUGER
        rrt.PLOT_TREE = self.PLOT_TREE
        rrt.PLOT_TREE_FAIL = self.PLOT_TREE_FAIL
        rrt.CONNECT_DIST = 50
        
        # Goal. For now the center of the next region's bounding box        
        maxX = max(nextRegPoints[:,0])
        minX = min(nextRegPoints[:,0])
        maxY = max(nextRegPoints[:,1])
        minY = min(nextRegPoints[:,1])        
        goalPoint = array([(maxX + minX)/2.0, (maxY + minY)/2.0])
        currPoint = pose[:2]
        dirV = goalPoint - currPoint
        goalAngle = arctan2(dirV[1], dirV[0])
        goalPose = array([goalPoint[0], goalPoint[1], goalAngle])
        
        
        # Start plotting and generating the path
        self.plotter.clearPlot()
        
        if self.PLOT_REG:
            self.plotter.drawMap(rrtMap)
            self.plotter.drawStartAndGoalRobotShape(pose, goalPose, self.robot)
            
        rrtPath = rrt.getRRTDipoleControlPath(pose, goalPose)
    
        if self.PLOT_PATH:
            pathT = rrt.get2DPathRepresent(rrtPath)
            self.plotter.drawDipolePath2D(pathT, color='g', width=3)
            
        shortPath = rrt.getShortcutPathDipole(rrtPath)
        
        if self.PLOT_PATH:
            pathT = rrt.get2DPathRepresent(shortPath)
            self.plotter.drawDipolePath2D(pathT, color='r', width=3)
            
        # Update instance fields
        self.path = rrt.pathNodeToDipoles(shortPath)    # Convert to 3D vector
        self.nextWaypointIndex = 0
        
    def closeEnough(self, pose1, pose2):
        """ Returns true if pose1 and pose2 are within a threshold distance
        and angle difference.
        """
        DIST_THR = 10
        ANG_THR = 10
#         ANG_THR = .4
        
        dist = linalg.norm(pose1[:2] - pose2[:2])
        angDiff = abs(pose1[2] - pose2[2])
        
        if dist < DIST_THR and angDiff < ANG_THR:
            return True
        else:
            return False
        
        
        
        
        
        
    
    
    