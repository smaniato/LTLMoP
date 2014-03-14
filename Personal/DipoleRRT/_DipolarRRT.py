
from __future__ import division

from random import random
import numpy as np
from numpy.linalg import norm
from time import clock
from _DipolarCLoopControl import diffAngles
from math import atan2
from Queue import PriorityQueue

# TODO: CHANGE TANH IN CONTROLLER

DEBUG = False
DEBUG_PLT = False
    
try:
    import _PlotRRT
    plotter_avail = True
except:
    print "Plotter disabled."
    plotter_avail = False
    
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
        newNode = Node(self.position, self.orientation, self.parent)
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


class DipolarRRT:            
    
    def __init__(self, robot, rrtMap, controller, plotter=None, plottTree=True):
        """ An RRT* path planner that takes into account orientation requirements.

        """
        
        # Settings
        self.colCheckInter = .05        # Interval for checking collisions (time)
        self.stepSize = 1               # Step size for growing tree (distanceNodeE2)
        self.sampleGoalProb = .05       # Probability of sampling the goalNode
        self.closeEnoughEucl = .25
        self.closeEnoughEucl2 = self.closeEnoughEucl * self.closeEnoughEucl
        self.closeEnoughAng = .5
        
        # Store parameters
        self.robot = robot.copy()
        self.rrtMap = rrtMap          # TODO: MAKE A MAP COPY
        self.controller = controller    # Dipolar controller
        
        if plottTree and plotter is not None:
            self.plotting = True
            self.plotter = plotter
        elif plottTree and plotter is None and plotter_avail:
            self.plotting = True
            self.plotter = _PlotRRT.RRTPlotter()
        else:
            self.plotting = False
            self.plotter = None
        
        self.goalNode = None
        self.nodeList = []
        
    def saveObject(self, name, obj):
        import cPickle
        with open(name, "wb") as f:
            cPickle.dump(obj, f)
    
    def loadObject(self, name):
        import cPickle
        with open(name, "rb") as f:
            return cPickle.load(f)
        
    def setCloseEnough(self, euclidean, angle):
        self.closeEnoughEucl = euclidean
        self.closeEnoughEucl2 = self.closeEnoughEucl * self.closeEnoughEucl
        self.closeEnoughAng = angle        
       
    def sampleFree(self, isGoalReg=False):
        if random() < self.sampleGoalProb:
            if isGoalReg:
                goalPose = self.rrtMap.sampleGoal(self.robot)
                return Node.nodeFromPose(goalPose)
            else:
                return self.goalNode
        else:
            randPose = self.rrtMap.samplePose(self.robot)
            return Node.nodeFromPose(randPose)
     
    def distancePointE2(self, p1, p2):
        """ Returns distance squared for efficiency
        """
        x1, y1 = p1
        x2, y2 = p2
        x = x1 - x2
        y = y1 - y2
        return x*x + y*y 
     
    def distanceNodeE2(self, fromNode, toNode):
        """ Returns distance squared for efficiency
        """
        return self.distancePointE2(fromNode.position, toNode.position)
    
    def sortByDistancesE2(self, fromNodes, toNode):
        """ Returns the nodes ordered by distances along with the 
        corresponding distances.
        """
        numNodes = len(fromNodes)
        distances = map(self.distanceNodeE2, fromNodes, [toNode]*numNodes)
        nodeAndDist = zip(fromNodes, distances)
        sortedList = sorted(nodeAndDist, key=lambda x: x[1])
        nodes, distances = zip(*sortedList)
        return nodes, distances
    
    def closeEnoughNode(self, fromNode, toNode):
        return self.closeEnoughPose(fromNode.getPose(), toNode.getPose())
               
    def closeEnoughPose(self, fromPose, toPose):
        ang = abs(diffAngles(fromPose[2], toPose[2]))
        dist2 = self.distancePointE2(fromPose[:2], toPose[:2])
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
        getControls = self.controller.getControls
        integrateForwards = self.controller.integrateForwards
        isCollisionFree = self.rrtMap.isCollisionFree
        moveRobotTo = self.robot.moveRobotTo
        robot = self.robot
        DT = self.colCheckInter
        
        # TODO: REMOVE
        timeout=100
        
        timeStart = clock()
        while (clock() - timeStart) < timeout:
            u, w = getControls(poseCurr, poseDes, posePrev, DT)
            posePrev = poseCurr
            poseCurr = integrateForwards(posePrev, u, w, DT)
            path.append(poseCurr)
            
            # Check for collision
            moveRobotTo(poseCurr)
            if not isCollisionFree(robot):
                return None
            
            # If reached goal return the final state
            if self.closeEnoughPose(poseCurr, poseDes):
                return path
            
        # Timed out. This warning can be removed if bothersome.
        print "DEBUG: Warning getDipoleToDipolePath timed out."
        
        return None
    
    def nodesToPoses(self, nodeList):
        return [node.getPose() for node in nodeList]

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
        distances = [self.distancePointE2(path[i][:2], path[i+1][:2]) for i in range(len(path)-1)]
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
        NUM_CONSIDERED = 40
        
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
            
            # TODO: REMOVE
            if dist == 0:
                print "Here"
            
            currNode = node
            currP = currNode.getPosition()
            for _ in range(numIntervals):
                currP += dirV
                nextNode = Node(currP, dirAng, currNode)
                
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
        prevLen = len(self.nodeList)
        
        # Sample a new state and try to extend
        randNode = self.sampleFree()
        self.extendTree(self.nodeList, randNode)
        
        # Plot or debug
        numNew = len(self.nodeList) - prevLen
        if self.plotting and numNew > 0:
            self.plotter.drawTree(self.nodeList[-numNew:])
            if DEBUG_PLT:
                self.plotter.pause(.0001)
                
        
        if numNew > 0:
            if self.closeEnoughNode(self.nodeList[-1], self.goalNode):
                return self.nodeList[-1]
            else:
                return None
        else:
            return None
        
    
    def iterateGoalReg(self):
        """ Go through one iteration of the RRT algorithm. Return true if 
        the goal regions was found.
        """
        prevLen = len(self.nodeList)
        
        # Sample a new state and try to extend
        randNode = self.sampleFree(isGoalReg=True)
        self.extendTree(self.nodeList, randNode)
        
        # Plot or debug
        numNew = len(self.nodeList) - prevLen
        if self.plotting and numNew > 0:
            self.plotter.drawTree(self.nodeList[-numNew:])
            if DEBUG_PLT:
                self.plotter.pause(.0001)
        
        if numNew > 0:
            for node in self.nodeList[-numNew:]:
                self.robot.moveTo(node.getPose())
                if self.rrtMap.isInGoal(self.robot):
                    return node
                return None            
        else:
            return None
    
    def getNodePathFromLastNode(self, node):
        path = []
        while node is not None:
            path.append(node)
            node = node.parent
        path.reverse()
        return path

    def getNodePath(self, fromPose, goalReg=False, toPose=None, K=500):
        startNode = Node.nodeFromPose(fromPose, None)
        self.nodeList = [startNode]
        if goalReg:
            self.goalNode = None
            iterate = self.iterateGoalReg
        else:
            self.goalNode = Node.nodeFromPose(toPose, None)
            iterate = self.iterate
        
        # Iterate until path is found
        foundPath = False
        for i in range(K):
            if DEBUG:
                print "Iteration: ", i
            foundGoal = iterate()
            if foundGoal is not None:
                return self.getNodePathFromLastNode(foundGoal)
        
        return None
        
        
    def getDijkSmooth(self, path):
        """ Use the shortcut Dijkstra to return a shorter path in the form of
        a list of Nodes.
        
        :param path: A list of Nodes
        """
        numEdges = 0                # Keep track of number of edges calculated
                
        # Setup
        numNodesTotal = len(path)
        goalNode = path[-1]
        
        nodeIQueue = PriorityQueue()        # Node to expand (distance, nodeIndex)
        visited = [False]*numNodesTotal
        distances = {}                      # (nodeIndex:distanceFromStart)
        
        # Initialize 
        nodeIQueue.put((0,0))
        distances[0] = 0
        path[0].parent = None
        
        while not nodeIQueue.empty():
            currNodeI = nodeIQueue.get()[1]
            if visited[currNodeI]:
                continue
            currDist = distances[currNodeI]
            currNode = path[currNodeI]
            visited[currNodeI] = True
            
            # Check if reached Goal
            if currNode == goalNode:
                
                print "ShortcutDipole numEdges: " + str(numEdges)
                
                shortPath = []
                while currNode is not None:
                    shortPath.append(currNode)
                    currNode = currNode.parent
                shortPath.reverse()
                return shortPath
            
            # Go through all the non visited nodes
            for neighborI in range(numNodesTotal):
                if visited[neighborI]:
                    continue
                neighNode = path[neighborI]
                
                numEdges += 1
                
                # Calculate alternative distance 
                pathT = self.getDipolarPath(currNode, neighNode)
                if pathT is None:
                    continue
                pathT += [neighNode.getPose()]
                distAlternative = currDist + self.get2DPathLengthE2(pathT)
                
                # If shorter than recorded update
                if distAlternative < distances.get(neighborI, float('inf')):
                    nodeIQueue.put((distAlternative, neighborI))
                    distances[neighborI] = distAlternative
                    neighNode.parent = currNode
                   
        # No path was found 
        return None 
        
        
        
        
    
    
    