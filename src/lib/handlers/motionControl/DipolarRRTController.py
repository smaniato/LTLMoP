#!/usr/bin/env python
"""
==================================================
DipolarController.py - Dipolar Motion Controller
==================================================

This class uses a dipolar field along with an RRT to generate paths for the
robot as well as the linear and angular velocities to drive them.
"""

# TODO: REMOVE SCALLED ROBOT ADDITION
# TODO: CLOSE ENOUGH CAUSES ISSUES WITH REACHING REGION WITHOUT PROPPER POSES

from __future__ import division

import logging
import Polygon as PolygonImp
PolygonImp.setTolerance(0.01)
from Polygon import Polygon, Shapes as pShapes
import cPickle as pickle
from time import clock
from math import pi

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

"""
TODO: I think the dipolar controller should be in the drive handler
TODO: Close enough parameter can cause problems if path planner stops but
    is not actually inside of the region... Position and or theta wise. 
    At some point may have to make distinction between the point we are 
    driving towards and the point that was "close enough" to that point. 
"""

class motionControlHandler:
    def __init__(self, proj, shared_data, robotType, nodeDistInter, linearGain, angularGain,
                 plotTree, plotLTLMoPPath, plotRegion):
        """
        An RRT with dipoles for finding path in regions with theta constraints. 
        
        robotType (int): The robot shape to be used. Default is circle with radius .1 (default=0)
        nodeDistInter (float): The max euclidean distance between nodes. (default=1)
        linearGain (float): The k1 gain for the dipolar closed loop controller. (default=0.5)
        angularGain (float): the k2 gain for the dipolar closed loop controller. (default=1.5)
        plotTree (bool): Plot the RRT Live
        plotLTLMoPPath (bool): Plot paths after calculation 
        plotRegion (bool): Plot the current and next region
        """        
        # Settings
        self.DEBUG = False           # Print statements for debugging
        self.DEBUG_PLT = False        # If using a debugger. Matplotlib workaround
        self.PLOT_REG = plotRegion       # Plot the current and next region
        self.PLOT_TREE = plotTree      # Plot the RRT live
        self.PLOT_PATH = plotLTLMoPPath       # Plot path generated
        self.PLOT_TREE_FAIL = True # Plot the RRT if it fails to find a path
        simulation = True
        self.logName = "LogTest"            # Then name to log data under
                                            # Set to None to turn off    
        
        # Get references to handlers we'll need to communicate with
        self.drive_handler = proj.h_instance['drive']
        self.pose_handler = proj.h_instance['pose']
        
        # Plotter
        if plotter_avail:
            # TODO: MAKE PARAMETER?
            self.plotter = RRTPlotter(invertY=simulation)
            self.plotter.ion()    
            self.plotTree = plotTree or plotLTLMoPPath or plotRegion 
        else: 
            self.plotter = None         
            self.plotTree = False   
            
#         # TODO: REMOVE USED TO GET ORDERING AND HARD CODING THETAS
#         self.showEachRegion(proj)
        
        # Get a list of (polygon, constraint) pairs
        self.regions = self.getConstrainedRegions(proj)
        
        # Log information. Start with map. Clear others.
        if self.logName is not None:
            with open(self.logName+"Map", "wb") as f:
                pickle.dump(self.regions, f) 
            with open(self.logName+"Pose", "w"): pass
            with open(self.logName+"Plan", "w"): pass
        
        # Dipolar controller
        self.dipController = DipolarController(k1=linearGain, k2=angularGain)
        self.prevPose = self.pose_handler.getPose()
        self.dT = 1/20.0    # The time elapsed since the call to controller
        
        # RRT Variables
        self.robot = self.getRobot(robotType)
        # TODO: MAP IS NONE. FINISH TODO IN _DipolarRRT
        self.rrt = DipolarRRT(self.robot, None, self.dipController, 
                              plotter=self.plotter, plotTree=plotTree)
        self.rrt.setCloseEnough(self.robot.radius, .2)
        self.rrt.stepSize = nodeDistInter
        self.rrt.fwdIntegrTime = self.dT
        
        _DipolarRRT.DEBUG = self.DEBUG
        _DipolarRRT.DEBUG_PLT = self.DEBUG_PLT
        _DipolarRRT.PLOT_TREE_FAIL = self.PLOT_TREE_FAIL
        
        # Keep track of path
        self.path = None
        self.nextDipoleIndex = None
        self.storedNextReg = None
        
        # TODO CLEAN UP
        self.closeExecEucl = 1
        self.closeExecEucl2 = self.closeExecEucl * self.closeExecEucl
        self.closeExecAng = 2

    def gotoRegion(self, current_reg, next_reg, last=False):
        """ Returns ``True`` if we've reached the next region. 
        Uses the generated path and dipoles to go to the next 
        dipole. If the desired region changes or no path
        currently exists, it will create one using the RRT
        """
#         self.drive_handler.setVelocity(.000, .001)
#         return False

        # Find our current configuration
        pose = self.pose_handler.getPose()

        # Log pose
        if self.logName is not None:
            with open(self.logName+"Pose", 'ab') as f:
                pickle.dump((pose, clock()), f)
        
        if current_reg == next_reg and not last:
            # No need to move!
            self.drive_handler.setVelocity(0, 0)  # So let's stop
            return True  # Already there
        
        # Check if a new path needs to be calculated
        if self.path is None or self.storedNextReg != next_reg:
            self.drive_handler.setVelocity(0, 0)
            self.findAndSetPath(current_reg, next_reg)
            self.storedNextReg = next_reg
            
        # Check if reached dipole
        while self.closeEnoughPose(pose, self.path[self.nextDipoleIndex]):
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
        
        logging.debug("\nPose: {} \nNext: {} \nVel: {}".format(pose, 
                      self.path[self.nextDipoleIndex], (v, w)))
        
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
        constMap = RRTMapConst(self.robot, [currPoly, nextPoly], 
                               [currConst, nextConst], 
                               [False, True])
        
        pose = self.pose_handler.getPose()
        
        # TODO: THIS IS KIND OF GHETTO 
        self.rrt.rrtMap = constMap
        
        # Start plotTree and generating the path if desired
        if self.plotTree:
            self.plotter.clearPlot()
        
        if self.PLOT_REG and self.plotTree:
#             self.plotter.drawMap(constMap)
            self.plotter.drawPolygon(currPoly, color='k', width=3)
            self.plotter.drawPolygon(nextPoly, color='g', width=3)
            
        if self.DEBUG:
            print "Getting RRTPath"
            
        nodePath = self.rrt.getNodePath(pose, K=2000)
        
        if nodePath is None:
            raise Exception("Could not find path in given iterations")
    
        if self.PLOT_PATH:
            trajectory = self.rrt.nodesToTrajectory(nodePath)
            self.plotter.drawPath2D(trajectory, color='g', width=3)
            
        if self.DEBUG:
            print "Getting shorter path"
            
        shortPath = self.rrt.applyDijkSmooth(nodePath)
        
        if self.PLOT_PATH:
            trajectory = self.rrt.nodesToTrajectory(shortPath)
            self.plotter.drawPath2D(trajectory, color='r', width=3)
            
        # Log planned trajectory
        if self.logName is not None: 
            trajectory = self.rrt.nodesToTrajectory(shortPath)
            with open(self.logName+"Plan", "ab") as f:
                pickle.dump(trajectory, f)  
                
#         if self.DEBUG:
#             pathT = self.rrt.get2DPathRepresent(shortPath)
#             print "Length of pathT=", len(pathT)
#             self.plotter.drawRobotOccupiedPath(pathT, self.robot, color='b', width=1)
            
        # Update instance fields
        self.path = self.rrt.nodesToPoses(shortPath)    # Convert to dipole list
        self.nextDipoleIndex = 0
        self.storedNextReg = next_reg        
    
    def getConstrainedRegions(self, proj):
        """ Returns a list of (polygon, theta constraint) pairs 
        """
        mapFun = proj.coordmap_map2lab
        regions = [self.getRegionPolygon(region, mapFun) 
                   for region in proj.rfi.regions]
        
        # TODO: THIS INFO SHOULD COME FROM REGIONS
        # Pair with theta constraints
        from math import pi
        # No constraints
#         return [(r, (0, pi)) for r in regions]
        # Z Shape map
#         thetaConstraints = [(0, pi/4), (0, pi), (0, pi), (pi/2, pi/4)]
        # Take footage map
        thetaConstraints = [(-pi/2, pi/4),       # r2 
                            (0, pi),            # r1 
                            (0, pi),            # r3
                            (pi, pi/10),      # Upload
                            (-pi/2, pi)      # Charge
                            ]
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
        
    def getRobot(self, robotType):
        """ Returns a predefined RRTRobot """
        # Simulated Circular robot (For when lab to map matrix is the identity)
        if robotType == 0:
            return RRTRobot.circularRobot([0,0,0], 10)
        
        # Create
        elif robotType == 1:
#             return RRTRobot.circularRobot([0,0,0], .01)  
            return RRTRobot.circularRobot([0,0,0], .1)    
        
        # Create
        elif robotType == 2: 
            shape = pShapes.Rectangle(.28, .28)
            shape.shift(-.14, -.14)
            return RRTRobot((0, 0, 0), shape, radius=.14)
        
        else:
            msg = "ERROR: DipolarRRTController - Undefined robot type."
            raise Exception(msg)
        
    #===========================================================================
    # Additional/Helper Methods
    #===========================================================================
    
    def diffAngles(self, angle1, angle2):
        """ Returns difference between -pi and pi. Relative to angle2.
        """
        return (angle1 - angle2 + pi)%(2*pi) - pi
        
    def closeEnoughPose(self, fromPose, toPose):
        ang = abs(self.diffAngles(fromPose[2], toPose[2]))
        v = toPose[:2] - fromPose[:2]
        dist2 = v[0]*v[0] + v[1]*v[1]
        if (ang < self.closeExecAng and dist2 < self.closeExecEucl2):
            return True
        else:
            return False
        
    def showEachRegion(self, proj):
        """ For debugging purposes. Will plot each region one by one in the
        order in which they apear in the list.
        """
        mapFun = proj.coordmap_map2lab
        regions = [self.getRegionPolygon(region, mapFun) 
                   for region in proj.rfi.regions]
        
        self.plotter.ioff()
        for r in regions:
            self.plotter.clearPlot()
            self.plotter.drawPolygon(r)
            self.plotter.show()
            
        if self.plotTree:
            self.plotter.ion()
    
    