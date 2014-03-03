from random import random
from time import clock

from _PlotRRT import RRTPlotter
import Polygon.Shapes as pShapes
from _RRTMapAndRobot import RRTMap, RRTMapConst, RRTRobot
import matplotlib.pyplot as plt
import numpy as np
import _DipolarCLoopControl
# import DipoleRRTStar, RRTStar
import RRTStar, DipoleRRTStar, _DipolarRRT

class TestRRT:    
    
    def __init__(self):
        self.DEBUGER = False            # Debugger is on
        self.PLOT_TREE = True           # Live tree
        self.PLOT_RRT_PATH = True
        self.PLOT_SHORT_PATH = False
        self.PLOT_SHORT_PATH = True
        self.PLOT_TREE_FAIL = True      # Plot final tree if it times out
        
    def getSampleMapRRT(self, mapIndex):
        """ Return a (startPose, endPose, map) .Map index defines which of the 
        maps will be returned.
        
        0: Map with random object positions. No path guaranteed.
        1: Simple box map
        2: Box map with box obstacle
        3: Map with multiple obstacles.
        """
        startPose = None
        endPose = None
        boundary = None
        allObstacles = []
    
        # MAP Random
        if mapIndex == 0:
            startPose = np.array([1, 1, 0])
            endPose = np.array([19, 19, np.pi])
             
            boundary = pShapes.Rectangle(20, 20) 
            
            obstacle1 = pShapes.Rectangle(5, 5)
            obstacle1.shift(random()*20, random()*20)
            obstacle2 = pShapes.Rectangle(5, 8)
            obstacle2.shift(random()*20, random()*20)
            obstacle3 = pShapes.Rectangle(9, 3)
            obstacle3.shift(random()*20, random()*20)
            allObstacles = [obstacle1, obstacle2, obstacle3]
    
        # MAP 1
        elif mapIndex == 1:
            startPose = np.array([1, 1, 0])
            endPose = np.array([8, 8, np.pi])
              
            boundary = pShapes.Rectangle(9, 9) 
            
            allObstacles = []
            
        # MAP 2
        elif mapIndex == 2:
            startPose = np.array([1, 1, 0])
            endPose = np.array([8, 8, np.pi])
              
            boundary = pShapes.Rectangle(9, 9) 
            
            obstacle1 = pShapes.Rectangle(5, 5)
            obstacle1.shift(2, 2)
            allObstacles = [obstacle1]
        
        # MAP 3
        elif mapIndex == 3:
            startPose = np.array([1, 1, 0])
            endPose = np.array([19, 19, np.pi])
             
            boundary = pShapes.Rectangle(20, 20) 
            
            obstacle1 = pShapes.Rectangle(5, 5)
            obstacle1.shift(2, 2)
            obstacle2 = pShapes.Rectangle(5, 8)
            obstacle2.shift(12, 6)
            obstacle3 = pShapes.Rectangle(9, 3)
            obstacle3.shift(0, 12)
            allObstacles = [obstacle1, obstacle2, obstacle3]
                
        else:
            return None
                
        testMap = RRTMap(boundary, allObstacles)    
        return startPose, endPose, testMap

    def getSampleConstMap(self, index):
        from math import pi
        
        polygons = []
        restrictions = [] 
        startPose = None
        endPose = None
        
        if index == 1:
            
            p = pShapes.Rectangle(4, 4)
            p.shift(0, 0)
            r = (0, pi)
            polygons.append(p)
            restrictions.append(r)
            
            p = pShapes.Rectangle(8, 2)
            p.shift(4, 0)
            r = (-pi/2, pi/4)
            polygons.append(p)
            restrictions.append(r)
            
            p = pShapes.Rectangle(8, 2)
            p.shift(4, 2)
            r = (0, pi/4)
            polygons.append(p)
            restrictions.append(r)
            
            p = pShapes.Rectangle(4, 4)
            p.shift(12, 0)
            r = (0, pi)
            polygons.append(p)
            restrictions.append(r)
            
            p = pShapes.Rectangle(4, 4)
            p.shift(12, -4)
            r = (0, pi/4)
            polygons.append(p)
            restrictions.append(r)
            
            p = pShapes.Rectangle(4, 4)
            p.shift(16, -4)
            r = (0, pi)
            polygons.append(p)
            restrictions.append(r)
            
            p = pShapes.Rectangle(8, 4)
            p.shift(12, -8)
            r = (-pi, pi/4)
            polygons.append(p)
            restrictions.append(r)
            
            startPose = np.array((2, 2, 0))
            endPose = np.array((14, -6, pi))
        
        else:
            return None
        
        polyMap = RRTMapConst(polygons, restrictions)
        return startPose, endPose, polyMap

    def testRRTStar(self):
        RRTStar.DEBUG = False
        
        plt.ion()
            
        startPose, endPose, testMap = self.getSampleMapRRT(3)

        robot = RRTRobot.circularRobot((0,0,0), .5)
        
        plotter = RRTPlotter()
        plotter.drawMap(testMap)
        plotter.drawStartAndGoalRobotShape(startPose, endPose, robot)

        planner = RRTStar.RRTStar(robot, testMap, gama=20, plotter=plotter)
        planner.runKIterations(startPose[:2], endPose[:2], 500)
            
        print "Showing Final Results"
        plt.ioff()
        plt.show()    
        
    def testDipolarRRTStar(self, plotting=True):
        DipoleRRTStar.DEBUG = False
        
        if plotting:
            plt.ion()
            
        startPose, endPose, testMap = self.getSampleMapRRT(3)

        robot = RRTRobot.circularRobot((0,0,0), .5)
        controller = _DipolarCLoopControl.DipolarController()
        
        plotter = RRTPlotter()
        if plotting:
            plotter.drawMap(testMap)
            plotter.drawStartAndGoalRobotShape(startPose, endPose, robot)

        planner = DipoleRRTStar.DipolarRRTStar(robot, testMap, controller, 
                                               gama=40, plotter=plotter, 
                                               plotting=plotting)
        planner.runKIterations(startPose, endPose, 1000)
            
        if plotting:
            print "Showing Final Results"
            plt.ioff()
            plt.show()    
    
    def testDipolarRRT(self, plotting=True):
        _DipolarRRT.DEBUG = True
        _DipolarRRT.DEBUG_PLT = False
        
        if plotting:
            plt.ion()
            
        startPose, endPose, polyMap = self.getSampleConstMap(1)

        robot = RRTRobot.circularRobot((0,0,0), .5)
        controller = _DipolarCLoopControl.DipolarController()
        
        plotter = RRTPlotter()
        if plotting:
            plotter.drawMapConst(polyMap)
            plotter.drawStartAndGoalRobotShape(startPose, endPose, robot)

        planner = _DipolarRRT.DipolarRRT(robot, polyMap, controller, 
                                               plotter=plotter, 
                                               plotting=plotting)
        
        nodePath = planner.getNodePath(startPose, endPose, 1000)
        if plotting and nodePath is not None:
            dipPath = planner.nodesToTrajectory(nodePath)
            plotter.drawPath2D(dipPath, color='g', width=2)
        
        if nodePath is not None:
            dijkNodePaht = planner.getDijkSmooth(nodePath)
            if plotting and dijkNodePaht is not None:
                dipPath = planner.nodesToTrajectory(dijkNodePaht)
                plotter.drawPath2D(dipPath, color='r', width=2)
            
        if plotting:
            print "Showing Final Results"
            plt.ioff()
            plt.show()
            
    def testController(self):
        poseCurr = (1,2,3)
        poseDes = (4,5,1)
        posePrev = (3,2,1)
        
        c = _DipolarCLoopControl.DipolarController()
        
#         [c.getFieldAt1(poseCurr, poseDes) for _ in xrange(10000)]
#         [c.getFieldAt2(poseCurr, poseDes) for _ in xrange(10000)]
#         [c.getFieldAt3(poseCurr, poseDes) for _ in xrange(10000)]

#         [c.getControlls(poseCurr, poseDes, posePrev, .01) for _ in xrange(10000)]
#         [c.getControlls2(poseCurr, poseDes, posePrev, .01) for _ in xrange(10000)]
    
#         [c.integrateForwards(posePrev, 2, 0, .01) for _ in xrange(10000)]

if __name__ == "__main__":

    print "Starting"
    timeS = clock()
       
    test = TestRRT()
#     test.testRRTStar() 
#     test.testDipolarRRTStar(True)  
    test.testDipolarRRT(plotting=True)
    
#     import cProfile as Profile
#     Profile.run("test.testDipolarRRTStar(False)")   
#     Profile.run("test.testController()")
      
    print "Done: " + str(clock()-timeS)
