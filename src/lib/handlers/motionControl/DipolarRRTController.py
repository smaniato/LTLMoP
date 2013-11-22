#!/usr/bin/env python
"""
==================================================
skeletonController.py - Skeleton Motion Controller
==================================================
"""
from Polygon import Polygon
from numpy import *
from DipolarRRT import RRTMap, RRTPlotter, RRTRobot, DipolarRRT, DipolarController

class motionControlHandler:
    def __init__(self, proj, shared_data):
        # Settings
        robotRadius = .2
        self.DEBUGER = False        # If using a debugger. Matplotlib workaround
        self.PLOT_REG = True       # Plot the current and next region
        self.PLOT_TREE = True      # Plot the RRT live
        self.PLOT_TREE_FAIL = True # Plot the RRT if it fails to find a path
        self.PLOT_PATH = True       # Plot path generated
        
        # Get references to handlers we'll need to communicate with
        self.drive_handler = proj.drive_handler
        self.pose_handler = proj.pose_handler
        
        # Get information about regions
        self.rfi = proj.rfi
        self.coordmap_map2lab = proj.coordmap_map2lab
        
        # RRT Variables
        self.path = None
        self.nextWaypointIndex = None
        self.robot = RRTRobot.circularRobot([0, 0, 0], robotRadius)
        
        # Dipolar controller
        self.diController = DipolarController()
        self.prevPose = self.pose_handler.getPose()
        self.dT = 1/15.0    # The time elapsed since the call to controller

    def gotoRegion(self, current_reg, next_reg):
        """
        Returns ``True`` if we've reached the destination region.
        """

        # TODO: Whats going on here
        if current_reg == next_reg and not last:
            # No need to move!
            self.drive_handler.setVelocity(0, 0)  # So let's stop
            return True  # Already there
        
        # Check if a new path needs to be calculated
        if self.path is None:
            self.drive_handler.setVelocity(0, 0)
            self.findAndSetPath(current_reg, next_reg)
            
        # Find our current configuration
        pose = self.pose_handler.getPose()
            
        # Check if reached waypoint
        if self.closeEnough(pose, self.path[self.nextWaypointIndex]):
            self.nextWaypointIndex += 1
            
        # Check end of path
        if self.nextWaypointIndex >= len(self.path):
            self.drive_handler.setVelocity(0, 0)
            return True  # Already there

        # Calculate a velocity vector in the *GLOBAL REFERENCE FRAME* 
        # that will get us on our way to the next region
        nextWaypoint = self.path[self.nextWaypointIndex]
        u, v = self.diController.getControlls(pose, nextWaypoint, 
                                              self.prevPose, self.dT)
        vx, vy = 0, 0

        # Pass this desired velocity on to the drive handler
        # It also gets the current heading of the robot
        self.drive_handler.setVelocity(vx, vy, pose[2])
        
        # TODO: Whats going on here
        # Figure out whether we've exited the current region
        if is_inside([pose[0], pose[1]], vertices):
            arrived = False
        else:
            arrived = True
        
        self.prevPose = pose    
        
        return arrived
    
    def findAndSetPath(self, current_reg, next_reg):
        """ Calculate the path to go from current_reg to next_reg. """
        
        # Current region
        currRegPoints = [x for x in self.rfi.regions[current_reg].getPoints()]
        currRegPoints = map(self.coordmap_map2lab, currRegPoints)
        currRegPoints = mat(currRegPoints).T 
        currRegPoly = Polygon(currRegPoints)
        
        # Next region
        nextRegPoints = [x for x in self.rfi.regions[next_reg].getPoints()]
        nextRegPoints = map(self.coordmap_map2lab, nextRegPoints)
        nextRegPoints = mat(nextRegPoints).T 
        nextRegPoly = Polygon(nextRegPoints)       
        
        # Prepare RRT
        workspace = currRegPoly + nextRegPoly
        rrtMap = RRTMap(workspace)
        
        pose = self.pose_handler.getPose()
        plotter = RRTPlotter()
        plotter.ion()                       # Turn on interactive mode
        
        rrt = DipolarRRT(rrtMap, self.robot, plotter)
        rrt.DEBUGER = self.DEBUGER
        rrt.PLOT_TREE = self.PLOT_TREE
        rrt.PLOT_TREE_FAIL = self.PLOT_TREE_FAIL
        
        # Goal. For now the center of the next region's bounding box
        maxX = max(nextRegPoints[:,0])
        minX = min(nextRegPoints[:,0])
        maxY = max(nextRegPoints[:,1])
        minY = min(nextRegPoints[:,1])
        goalPoint = array((maxX + minX)/2.0, (maxY + minY)/2.0)
        currPoint = pose[:2]
        dirV = goalPoint - currPoint
        goalAngle = arctan2(dirV[1], dirV[0])
        goalPose = array([goalPoint[0], goalPoint[1], goalAngle])
        
        # Start plotting and generating the path
        plotter.clearPlot()
        
        if self.PLOT_REG:
            plotter.drawMap(rrtMap)
            plotter.drawStartAndGoalRobotShape(pose, goalPose, self.robot)
            
        rrtPath = rrt.getRRTDipoleControlPath(pose, goalPose)
    
        if self.PLOT_PATH:
            pathT = rrt.get2DPathRepresent(rrtPath)
            plotter.drawDipolePath2D(pathT, color='g', width=3)
            
        shortPath = rrt.getShortcutPathDipole(rrtPath)
        
        if self.PLOT_PATH:
            pathT = rrt.get2DPathRepresent(shortPath)
            plotter.drawDipolePath2D(pathT, color='r', width=3)
            
        # Update instance fields
        self.path = rrt.pathNodeToDipoles(shortPath)    # Convert to 3D vector
        self.nextWaypointIndex = 0
        
    def closeEnough(self, pose1, pose2):
        """ Returns true if pose1 and pose2 are within a threshold distance
        and angle difference.
        """
        DIST_THR = .5
        ANG_THR = .4
        
        dist = linalg.norm(pose1[:2] - pose2[:2])
        angDiff = abs(pose1[3] - pose2[3])
        
        if dist < DIST_THR and angDiff < ANG_THR:
            return True
        else:
            return False
        
        
        
        
        
        
    
    
    