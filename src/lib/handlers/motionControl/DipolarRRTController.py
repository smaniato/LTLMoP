#!/usr/bin/env python
"""
==================================================
DipolarController.py - Dipolar Motion Controller
==================================================

This class uses a dipolar field along with an RRT to generate paths for the
robot as well as the linear and angular velocities to drive them.
"""

from __future__ import division

import logging
from Polygon import Polygon

# Import helpers
import sys, os
helperPath = os.path.join(os.getcwd(), "lib", "handlers", "motionControl", 
                          "DipoleRRTHelper")
sys.path.append(helperPath)
import _DipolarRRT
from _DipolarRRT import DipolarRRT
from _DipolarCLoopControl import DipolarController
from _RRTMapAndRobot import RRTMapConst, RRTRobot
try:
    from _PlotRRT import RRTPlotter
    plotter_avail = True
except:
    logging.info("Plotter disabled.")
    plotter_avail = False

# TODO: I think the dipolar controller should be in the drive handler

class motionControlHandler:
    def __init__(self, proj, shared_data, robotType, nodeDistInter, linearGain, angularGain,
                 plotTree, plotPath, plotRegion):
        """
        An RRT with dipoles for finding path in regions with theta constraints. 
        
        robotType (int): The robot shape to be used. Default is circle with radius .1 (default=0)
        nodeDistInter (float): The max euclidean distance between nodes. (default=1)
        linearGain (float): The k1 gain for the dipolar closed loop controller. (default=0.5)
        angularGain (float): the k2 gain for the dipolar closed loop controller. (default=1.5)
        plotTree (bool): Plot the RRT Live
        plotPath (bool): Plot paths after calculation 
        plotRegion (bool): Plot the current and next region
        """        
        # Settings
        self.DEBUG = False           # Print statements for debugging
        self.DEBUG_PLT = False        # If using a debugger. Matplotlib workaround
        self.PLOT_REG = plotRegion       # Plot the current and next region
        self.PLOT_TREE = plotTree      # Plot the RRT live
        self.PLOT_PATH = plotPath       # Plot path generated
        self.PLOT_TREE_FAIL = True # Plot the RRT if it fails to find a path
        
        # Get references to handlers we'll need to communicate with
        self.drive_handler = proj.h_instance['drive']
        self.pose_handler = proj.h_instance['pose']
        
        # Get a list of (polygon, constraint) pairs
        self.regions = self.getConstrainedRegions(proj)
        
        # Plotter
        if plotter_avail:
            self.plotter = RRTPlotter(invertY=True)
            self.plotter.ion()    
            self.plotting = plotTree or plotPath or plotRegion 
        else: 
            self.plotter = None         
            self.plotting = False    
        
        # Dipolar controller
        self.dipController = DipolarController(k1=linearGain, k2=angularGain)
        self.prevPose = self.pose_handler.getPose()
        self.dT = 1/20.0    # The time elapsed since the call to controller
        
        # RRT Variables
        self.robot = self.getRobot(robotType)
        # TODO: MAP IS NONE. FINISH TODO IN _DipolarRRT
        self.rrt = DipolarRRT(self.robot, None, self.dipController, 
                              plotter=self.plotter, plotTree=plotTree)
        self.rrt.setCloseEnough(self.robot.radius, .3)
        self.rrt.stepSize = nodeDistInter
        self.rrt.colCheckInter = self.dT
        
        _DipolarRRT.DEBUG = self.DEBUG
        _DipolarRRT.DEBUG_PLT = self.DEBUG_PLT
        _DipolarRRT.PLOT_TREE_FAIL = self.PLOT_TREE_FAIL
        
        # Keep track of path
        self.path = None
        self.nextDipoleIndex = None
        self.storedNextReg = None
        
#         # TODO: REMOVE 
#         self.showEachRegion(self.regions)

    def gotoRegion(self, current_reg, next_reg, last=False):
        """ Returns ``True`` if we've reached the next region. 
        Uses the generated path and dipoles to go to the next 
        dipole. If the desired region changes or no path
        currently exists, it will create one using the RRT
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
            
        # Check if reached dipole
        while self.rrt.closeEnoughPose(pose, self.path[self.nextDipoleIndex]):
            self.nextDipoleIndex += 1
            
            # Check end of path
            if self.nextDipoleIndex >= len(self.path):
                self.path = None
                self.drive_handler.setVelocity(0, 0)
                return True  # Already there

        # Calculate the linear and angular velocities
        nextDipole = self.path[self.nextDipoleIndex]
        v, w = self.dipController.getControls(pose, nextDipole, 
                                               self.prevPose, self.dT)
        self.drive_handler.setVelocity(v, w)
        self.prevPose = pose  
        
#         if self.DEBUG:
#             print "-----------------"
#             print "Current pose:", pose
#             print "Next waypoint:", nextDipole
#             print "V:", v, "    W:", w

        return False  
    
    def findAndSetPath(self, current_reg, next_reg):
        """ Calculate the path to go from current_reg to next_reg. 
        """        
        # Current and next regions
        currPoly, currConst = self.regions[current_reg]
        nextPoly, nextConst = self.regions[next_reg]
        
        # Prepare RRT
        constMap = RRTMapConst([currPoly, nextPoly], [currConst, nextConst], [False, True])
        
        pose = self.pose_handler.getPose()
        
        # TODO: THIS IS KIND OF GHETTO 
        self.rrt.rrtMap = constMap
        
        # Start plotting and generating the path if desired
        if self.plotting:
            self.plotter.clearPlot()
        
        if self.PLOT_REG and self.plotting:
#             self.plotter.drawMap(constMap)
            self.plotter.drawPolygon(currPoly, color='k', width=3)
            self.plotter.drawPolygon(nextPoly, color='g', width=3)
            
        if self.DEBUG:
            print "Getting RRTPath"
            
        nodePath = self.rrt.getNodePath(pose, goalReg=True, K=2000)
    
        if self.PLOT_PATH:
            trajectory = self.rrt.nodesToTrajectory(nodePath)
            self.plotter.drawPath2D(trajectory, color='g', width=3)
            
        if self.DEBUG:
            print "Getting shorter path"
            
        shortPath = self.rrt.getDijkSmooth(nodePath)
        
        if self.PLOT_PATH:
            trajectory = self.rrt.nodesToTrajectory(shortPath)
            self.plotter.drawPath2D(trajectory, color='r', width=3)
            
#         if self.DEBUG:
#             pathT = self.rrt.get2DPathRepresent(shortPath)
#             print "Length of pathT=", len(pathT)
#             self.plotter.drawRobotOccupiedPath(pathT, self.robot, color='b', width=1)
            
        # Update instance fields
        self.path = self.rrt.nodesToPoses(shortPath)    # Convert to dipole list
        self.nextDipoleIndex = 0
        self.storedNextReg = next_reg        
    
    def getConstrainedRegions(self, proj):
        mapFun = proj.coordmap_map2lab
        regions = [self.getRegionPolygon(region, mapFun) 
                   for region in proj.rfi.regions]
        
        # TODO: THIS INFO SHOULD COME FROM REGIONS
        # Pair with theta constraints
        from math import pi
#         return [(r, (0, pi)) for r in regions]
        thetaConstraints = [(0, pi/4), (0, pi), (0, pi), (pi/2, pi/4)]
        return zip(regions, thetaConstraints)
        
    def getRegionPolygon(self, region, mapFun):
        """ Takes in a regions.region object and a maping function to 
        return the regions's polygon
        """
        boundaryPoints = [x for x in region.getPoints()]
        boundaryPoints = map(mapFun, boundaryPoints)
        regionPoly = Polygon(boundaryPoints)
        
        # Uncomment to allow holes. Warning (results in no region overlap)
#         # Remove holes
#         for holeId in range(len(region.holeList)):
#             pointsT = [x for x in region.getPoints(hole_id=holeId)]
#             pointsT = map(mapFun, pointsT)
#             regionPoly -= Polygon(pointsT)
            
        return regionPoly        
    
    def showEachRegion(self, regions):
        """ For debugging purposes. Will plot each region one by one in the
        order in which they apear in the list.
        """
        self.plotter.ioff()
        for r, _ in regions:
            self.plotter.clearPlot()
            self.plotter.drawPolygon(r)
            self.plotter.show()
            
        if self.plotting:
            self.plotter.ion()
        
    def getRobot(self, robotType):
        """ Returns a predefined RRTRobot """
        # Simulated Circular robot (For when lab to map matrix is the identity)
        if robotType == 0:
            return RRTRobot.circularRobot([0,0,0], 10)
        
        # Create
        elif robotType == 1:
            return RRTRobot.circularRobot([0,0,0], .1)            
        
        else:
            msg = "ERROR: DipolarRRTController - Undefined robot type."
            raise Exception(msg)
        
        
        
    
    
    