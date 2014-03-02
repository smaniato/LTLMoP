
from __future__ import division

from random import random
import numpy as np
from numpy.linalg import norm
from time import clock
from _DipolarCLoopControl import diffAngles
from math import atan2

DEBUG = False
    
try:
    import _PlotRRT
    plotter_avail = True
except:
    print "Plotter disabled."
    plotter_avail = False

class DipolarRRTStar:
    
    class Node:
        def __init__(self, position, orientation=None, parent=None):
            self.position = np.array(position, dtype=float)
            self.orientation = orientation
            if orientation is not None:
                xyT = (self.position[0], self.position[1], self.orientation)
                self.xyTheta = np.array(xyT, dtype=float)
            else:
                self.xyTheta = None
                
            self.parent = parent
            self.costFromParent = None          
            self.costFromRoot = None
            
        @classmethod
        def nodeFromPose(cls, pose, parent=None):
            return cls(pose[:2], pose[2], parent)
        
        def copy(self):
            newNode = self.Node(self.position, self.orientation, self.parent)
            newNode.costFromParent = self.costFromParent
            newNode.costFromRoot = self.costFromRoot
            
        def setPosition(self, xy):
            self.position = xy[:2]
            if self.xyTheta is not None:
                self.xyTheta[:2] = xy[:2]
                        
        def getPosition(self):
            return np.array(self.position)
            
        def setPose(self, xyTheta):
            if self.xyTheta is None:
                self.xyTheta = np.array(xyTheta, dtype=float)
            else:
                self.xyTheta = xyTheta[:3]
            
        def getPose(self):
            return np.array(self.xyTheta)
        
        def dirVectFrom(self, fromNode):
            return self.position - fromNode.getPosition()
            
    
    def __init__(self, robot, polyMap, controller, plotter=None, plotting=True):
        """ An RRT* path planner that takes into account orientation requirements.

        :param robot: An RRTRobot object
        :param polyMap: An RRTMap object
        :param controller: A DipolarController object
        :param gama: The gama used for calculating neighbor radius
        :param plotter: An RRTPlotter object. If plotting is desired
        :param plotting: Enable and disable plotting
        """
        
        # Settings
        self.colCheckInter = .05        # Interval for checking collisions (time)
        self.stepSize = 1               # Step size for growing tree (distanceE2)
        self.sampleGoalProb = .10       # Probability of sampling the goalNode
        self.closeEnoughEucl = .25
        self.closeEnoughEucl2 = self.closeEnoughEucl * self.closeEnoughEucl
        self.closeEnoughAng = .5
        
        # Store parameters
        self.robot = robot.copy()
        self.polyMap = polyMap          # TODO: MAKE A MAP COPY
        self.controller = controller    # Dipolar controller
        
        if plotting and plotter is not None:
            self.plotting = True
            self.plotter = plotter
        elif plotting and plotter is None and plotter_avail:
            self.plotting = True
            self.plotter = _PlotRRT.RRTPlotter()
        else:
            self.plotting = False
            self.plotter = None
        
        self.goalNode = None
        self.nodeList = []
        
    def setCloseEnough(self, euclidean, angle):
        self.closeEnoughEucl = euclidean
        self.closeEnoughEucl2 = self.closeEnoughEucl * self.closeEnoughEucl
        self.closeEnoughAng = angle        
        
    def runKIterations(self, fromPose, toPose, K):
        # TODO: 
        pass
#         # Setup 
#         startNode = self.Node.nodeFromPose(fromPose) 
#         startNode.costFromParent = 0
#         startNode.costFromRoot = 0
#         
#         self.nodeList = [startNode]
#         self.goalList = []
#         
#         self.goalNode = toPose
#         self.iterations = 0
#         
#         # Run the desired number of iterations
#         for i in range(K):
#             print "Iteration: ", i
#             self.iterate()
#             
# #         # TODO: REMOVE
# #         self.goalList.append(self.nodeList[-1])
#             
#         # Show results
#         print "Num goals: ", len(self.goalList)
#         print "Num nodes: ", len(self.nodeList)
#         nodePath = self.getBestNodePathFromTree()
#         if nodePath is None:
#             print "No path found..."
#             return
#         if self.plotting:            
#             self.plotter.clearPlot()
#             self.plotter.drawMap(self.polyMap)
#             self.plotter.drawTree(self.nodeList)
#             
#             trajectory = self.nodesToTrajectory(nodePath)
#             self.plotter.drawPath2D(trajectory, color='g', width=2)
#             self.plotter.drawStartAndGoalPoints(fromPose, toPose)
       
    def sampleFree(self):
        # TODO:
        pass
#         if random() < self.sampleGoalProb:
#             randPose = self.goalNode
#         else:
#             # TODO: Change the way in which map is used
#             randPoint = self.polyMap.cFree.sample(random)
#             randAng = random() * 2 * np.pi 
#             randPose = (randPoint[0], randPoint[1], randAng)
#             
#         return self.Node.nodeFromPose(randPose)
     
    def distanceE2(self, fromNode, toNode):
        """ Returns distance squared for efficiency
        """
        x1, y1 = fromNode.position
        x2, y2 = toNode.positions
        x = x1 - x2
        y = y1 - y2
        return x*x + y*y 
    
    def sortByDistancesE2(self, fromNodes, toNode):
        """ Returns the nodes ordered by distances along with the 
        corresponding distances.
        """
        numNodes = len(fromNodes)
        distances = map(self.distanceE2, fromNodes, [toNode]*numNodes)
        nodeAndDist = zip(fromNodes, distances)
        sortedList = sorted(nodeAndDist, key=lambda x: x[1])
        nodes, distances = zip(*sortedList)
        return nodes, distances
    
    def closeEnough(self, fromNode, toNode):
        ang = abs(diffAngles(fromNode.orientation, toNode.orientation))
        dist2 = self.distanceE2(fromNode, toNode)
        if (ang < self.closeEnoughAng and dist2 < self.closeEnoughEucl2):
            return True
        else:
            return False
             
    # TODO: PICK GOOD TIMEOUT. REMOVE WARNING
    def getDipolarPath(self, fromNode, toNode, timeout=.5):
        """ Returns the path traveled as a list of 1x3 arrays or None if a 
        collision or timeout occurs. 
        """        
        poseCurr = fromNode.getPose()
        poseDes = toNode.getPose()
        posePrev = poseCurr
        path = [poseCurr]
        
        # Rename functions
        getControlls = self.controller.getControlls
        integrateForwards = self.controller.integrateForwards
        isCollisionFree = self.polyMap.cFree.covers
        moveRobotTo = self.robot.moveRobotTo
        robotShape = self.robot.shape
        DT = self.colCheckInter
        
        timeStart = clock()
        while (clock() - timeStart) < timeout:
            u, w = getControlls(poseCurr, poseDes, posePrev, DT)
            posePrev = poseCurr
            poseCurr = integrateForwards(posePrev, u, w, DT)
            path.append(poseCurr)
            
            # Check for collision
            moveRobotTo(poseCurr)
            if not isCollisionFree(robotShape):
                return None
            
            # If reached goal return the final state
            if self.closeEnough(poseCurr, poseDes):
                return path
            
        # Timed out. This warning can be removed if bothersome.
        print "DEBUG: Warning getDipoleToDipolePath timed out."
        
        return None

    def nodesToTrajectory(self, nodePath):
        paths = [self.getDipolarPath(nodePath[i], nodePath[i+1])
                 for i in range(len(nodePath) - 1)]
        
        # Concatenate all of the paths
        trajectory = []
        map(trajectory.extend, paths)
        return trajectory     
    
    def get2DPathLengthE2(self, path):
        """ Get the length of a path along the set of 2D waypoints. Use euclidean
        distance squared.
        
        :param path: A list of points or poses as numpy arrays.
        """
        def dist2(p1, p2):
            x1, y1 = p1
            x2, y2 = p2
            x = x1 - x2
            y = y1 - y2
            return x*x + y*y 
        
        distances = [dist2(path[i][:2], path[i+1][:2]) for i in range(len(path)-1)]
        return sum(distances)
    
    def hasCollision(self, fromNode, toNode):
        """ Check for collisions in intervals. 
        """
        if self.getDipolarPath(fromNode, toNode) is None:
            return True
        else:
            return False        
              
    def extendTree(self, fromNodes, toNode):
        """ Uses a version of the connect heuristic to extend the tree towards
        the toNode. 
        """
        NUM_CONSIDERED = 5
        
        # Sort the nodes to try to extend from.
        ordered = self.sortByDistancesE2(fromNodes, toNode)[0][:NUM_CONSIDERED]
        
        # Try to extend from nodes in order. Only try next if no progress.
        initialLen = len(fromNodes)
        for node in ordered:
            collided = False
            
            # Try to connect to dipole in intervals
            dirV = toNode.dirVectFrom(node)
            dirAng = atan2(dirV[1], dirV[0])
            dist = norm(dirV)
            numIntervals = int(dist/self.stepSize)
            dirV *= self.stepSize/dist
            
            currNode = node
            currP = currNode.position
            for _ in range(numIntervals):
                currP += dirV
                nextNode = self.Node(currP, dirAng, currNode)
                
                if self.hasCollision(currNode, nextNode):
                    collided = True
                    break
                
                fromNodes.append(nextNode)
                currNode = nextNode
                
            # Attempt to connect to toNode
            if not collided and not self.hasCollision(currNode, toNode):
                toNode.parent = currNode
                fromNodes.append(toNode)
                
            # Check if any progress was made
            if len(fromNodes) > initialLen:
                return
                   
    def iterate(self):
        """ Go through one iteration of the RRT algorithm. Return true if 
        the goal was found.
        """
        self.iterations += 1
        prevLen = len(self.nodeList)
        
        # Sample a new state and try to extend
        randNode = self.sampleFree()
        self.extendTree(self.nodeList, randNode)
        
        # Plot or debug
        numNew = len(self.nodeList) - prevLen
        if self.plotting and numNew > 0:
            self.plotter.drawTree(self.nodeList[-numNew:])
        
        return self.closeEnough(self.nodeList[-1], self.goalNode)

    def getPath(self, fromPose, toPose, K=500):
        # TODO: FINNISH
        startNode = self.Node.nodeFromPose(fromPose, None)
        self.nodeList = [startNode]
        self.goalNode = self.Node.nodeFromPose(toPose, None)
        
        for i in range(K):
            if DEBUG:
                print "Iteration: ", i
            foundPath = self.iterate()
            if foundPath:
                return
        
        
        
    
    
    