#!/usr/bin/env python
"""
==================================================
RRTTest
==================================================

This class can be used to test out individual 
components and see how some functions are used.
"""

from random import random
from time import clock

import Polygon.Shapes as pShapes
import _DipolarCLoopControl
import _DipolarRRT
from _PlotRRT import RRTPlotter
from _RRTMapAndRobot import RRTMap, RRTRobot
import cPickle as pickle
import matplotlib.pyplot as plt
import numpy as np


class TestRRT:    
        
    def saveObject(self, name, obj):
        import cPickle
        with open(name, "wb") as f:
            cPickle.dump(obj, f)
    
    def loadObject(self, name):
        import cPickle
        with open(name, "rb") as f:
            return cPickle.load(f)

    def getSampleMapGoalPoints(self, mapName, robot):
        startPose, rrtMap, goal = self.getSampleMapGoalReg(mapName, robot)
        
        if mapName == "ComplexConstraints":
            goal = [(14, -6, 3.14)]
            
        elif mapName == "Backup":
            goal = [(7, 3, 3.14)]
        
        else:
            raise Exception("Unknown Map...")
    
        return startPose, rrtMap, goal
    
    def getSampleMapGoalReg(self, mapName, robot):
        from math import pi
        
        polygons = []
        constraints = [] 
        areGoals = []
        startPose = None
        
        if mapName == "ComplexConstraints":
            
            p = pShapes.Rectangle(4, 4)
            p.shift(0, 0)
            r = (0, pi)
            polygons.append(p)
            constraints.append(r)
            areGoals.append(False)
            
            p = pShapes.Rectangle(8, 2)
            p.shift(4, 0)
            r = (-pi/2, pi/4)
            polygons.append(p)
            constraints.append(r)
            areGoals.append(False)
            
            p = pShapes.Rectangle(8, 2)
            p.shift(4, 2)
            r = (0, pi/4)
            polygons.append(p)
            constraints.append(r)
            areGoals.append(False)
            
            p = pShapes.Rectangle(4, 4)
            p.shift(12, 0)
            r = (0, pi)
            polygons.append(p)
            constraints.append(r)
            areGoals.append(False)
            
            p = pShapes.Rectangle(4, 4)
            p.shift(12, -4)
            r = (0, pi/4)
            polygons.append(p)
            constraints.append(r)
            areGoals.append(False)
            
            p = pShapes.Rectangle(4, 4)
            p.shift(16, -4)
            r = (0, pi)
            polygons.append(p)
            constraints.append(r)
            areGoals.append(False)
            
            p = pShapes.Rectangle(8, 4)
            p.shift(12, -8)
            r = (-pi, pi/4)
            polygons.append(p)
            constraints.append(r)
            areGoals.append(False)
            
            p = pShapes.Rectangle(3, 3)
            p.shift(12.5, -7.5)
            r = (0, pi)
            polygons.append(p)
            constraints.append(r)
            areGoals.append(True)            
            
            startPose = np.array((2, 2, 0))
            
        elif mapName == "SimpleConstraints":
            
            p = pShapes.Rectangle(16, 4)
            p.shift(0, 0)
            r = (0, pi)
            polygons.append(p)
            constraints.append(r)
            areGoals.append(False)
            
            p = pShapes.Rectangle(4, 16)
            p.shift(12, -12)
            r = (0, pi)
            polygons.append(p)
            constraints.append(r)
            areGoals.append(False)
            
            p = pShapes.Rectangle(2, 2)
            p.shift(14, -8)
            r = (0, pi/8)
            polygons.append(p)
            constraints.append(r)
            areGoals.append(True)                       
            
            startPose = np.array((2, 2, 0))            
            
        elif mapName == "LongZigZag":
            
            p = pShapes.Rectangle(8, 4)
            p.shift(0, 0)
            r = (pi/2, pi/4)
            polygons.append(p)
            constraints.append(r)
            areGoals.append(False)
            
            p = pShapes.Rectangle(4, 4)
            p.shift(8, 0)
            r = (pi/2, pi/4)
            polygons.append(p)
            constraints.append(r)
            areGoals.append(False)
            
            p = pShapes.Rectangle(2, 2)
            p.shift(9, 1)
            r = (pi/2, pi/4)
            polygons.append(p)
            constraints.append(r)
            areGoals.append(True)
                                   
            startPose = np.array((2, 2, pi/2))
            
        elif mapName == "ShortZigZag":
            
            p = pShapes.Rectangle(4, 4)
            p.shift(0, 0)
            r = (pi/2, pi/4)
            polygons.append(p)
            constraints.append(r)
            areGoals.append(False)
            
            p = pShapes.Rectangle(4, 4)
            p.shift(4, 0)
            r = (pi/2, pi/4)
            polygons.append(p)
            constraints.append(r)
            areGoals.append(False)
            
            p = pShapes.Rectangle(2, 2)
            p.shift(5, 1)
            r = (pi/2, pi/4)
            polygons.append(p)
            constraints.append(r)
            areGoals.append(True)
                                   
            startPose = np.array((2, 2, pi/2))
            
        elif mapName == "Backup":
            
            p = pShapes.Rectangle(6, 6)
            p.shift(0, 0)
            r = (0, pi)
            polygons.append(p)
            constraints.append(r)
            areGoals.append(False)
            
            p = pShapes.Rectangle(2, 2)
            p.shift(6, 2)
            r = (pi, pi/4)
            polygons.append(p)
            constraints.append(r)
            areGoals.append(True)
                                   
            startPose = np.array((3, 3, 0))  
        
        else:
            return None
    
        rrtMap = RRTMap(robot, polygons, constraints)
        
        goalRegions = [(polygons[i], constraints[i])
                       for i in range(len(polygons)) if areGoals[i]]
        goalPolygons, goalConst = zip(*goalRegions)
        goal = RRTMap(robot, goalPolygons, goalConst)
        
        return startPose, rrtMap, goal
    
    def testDipolarRRT(self, plotting=True):
#         mapName = "ComplexConstraints"
        mapName = "SimpleConstraints"
#         mapName = "LongZigZag"
#         mapName = "ShortZigZag"
#         mapName = "Backup"
        
#         mapFun = self.getSampleMapGoalPoints
        mapFun = self.getSampleMapGoalReg
        
        # Initialization and setup
        _DipolarRRT.DEBUG = True
        _DipolarRRT.DEBUG_PLT = False
        if plotting:
            plt.ion()
        else:
            _DipolarRRT.DEBUG_PLT = False
            _DipolarRRT.PLOT_TREE_FAIL = False
            
        robot = RRTRobot.circularRobot((0,0,0), .5)
        controller = _DipolarCLoopControl.DipolarController()

        startPose, rrtMap, goal = mapFun(mapName, robot)
        
        if plotting:
            plotter = RRTPlotter()
            plotter.drawMapConst(rrtMap)
            robot.moveTo(startPose)
            plotter.drawPolygon(robot.shape, color='r', width=2)
        else:
            plotter = None

        # Planner and path generation
        planner = _DipolarRRT.DipolarRRT(robot, controller, plotter=plotter, 
                                         plotTree=plotting)
#         planner.sampleGoalProb = .1
        nodePath = planner.getNodePath(startPose, rrtMap, goal, K=500)
        
        # Save tree 
        self.saveObject("tree", planner.tree)
        
        # Plot path
        if plotting and nodePath is not None:
            dipPath = planner.nodesToTrajectory(nodePath)
            plotter.drawPath2D(dipPath, color='g', width=2)
        
        # Smooth and plot
        if nodePath is not None:
            dijkNodePath = planner.applyDijkSmooth(nodePath)
            if plotting and dijkNodePath is not None:
                dipPath = planner.nodesToTrajectory(dijkNodePath)
                plotter.drawPath2D(dipPath, color='r', width=2)
                
        self.saveObject("path", nodePath)
            
        if plotting:
            print "Showing Final Results"
            plt.ioff()
            plt.show()
        
    def plotLTLMoPPath(self):
        regionsName = "LogTestMap"
        posesName = "LogTestPose"
        pathsName = "LogTestPlan"
        
        def loadAllPickles(f):
            l = []
            while True:
                try: l.append(pickle.load(f))
                except: return l
        
        with open(regionsName, 'rb') as f: 
            regions = pickle.load(f)
        with open(posesName, 'rb') as f: 
            poses = loadAllPickles(f)
        with open(pathsName, 'rb') as f:
            paths = loadAllPickles(f)
            
        # Trim data
        trim1, trim2 = 0, 9999
        trim1, trim2 = 840, 3900
        poses = poses[trim1:trim2]
        
        poses, posesT = zip(*poses)
        
        import matplotlib.pyplot as plt
             
        # 2D Path
        plotter = RRTPlotter(invertY=True)
         
        plotter.drawPath2D(poses, 'r', ls='-', width=3, draw=True)
        for p in paths:
            plotter.drawPath2D(p, color='b', ls='--', width=4, draw=True)
        plotter.drawPath2D(poses, 'r', ls='-', width=3, draw=True)
        for r in regions:
            plotter.drawPolygon(r[0], color='k', width=3, draw=True)
             
        plt.title("Robot Paths")
        plt.xlabel("X-Coordinate")
        plt.ylabel("Y-Coordinate")
        plt.legend(["Robot Path", "Planned Path"])
             
        plotter.show()
        
        # Theta
        from math import pi
        thetas = [(0 - p[2])%(2*pi) for p in poses]
        plt.plot(posesT, thetas, 'b', linewidth=2)
        plt.plot(posesT, [pi/4]*len(posesT), 'g--', linewidth=2)
        plt.plot(posesT, [3*pi/4]*len(posesT), 'g--', linewidth=2)
        plt.ylim([.5, pi])
        plt.title("Robot Orientation in r2")
        plt.xlabel("Time (s)")
        plt.ylabel("Theta (radians)")
        plt.legend(["Robot Orientation", "Orientation Requirement"])
        plt.show()
        
        
if __name__ == "__main__":

    print "Starting"
    timeS = clock()
    
    test = TestRRT()
    test.testDipolarRRT(plotting=True)

#     test.plotLTLMoPPath()
    
#     import cProfile as Profile
#     Profile.run("test.testDipolarRRTStar(False)")   
#     Profile.run("test.testDipolarRRT()")
      
    print "Done: " + str(clock()-timeS)
