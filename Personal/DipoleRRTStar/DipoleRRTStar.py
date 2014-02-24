
from __future__ import division

from random import random
import numpy as np
from numpy.linalg import norm
from time import clock

DEBUG = False

# TODO: Find a good KDTree implementation
# from scipy.spatial import KDTree as Tree
    
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
            
    
    def __init__(self, robot, polyMap, controller, gama=1.5, plotter=None, plotting=True):
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
        self.stepSize = 2               # Step size for growing tree (distance)
        self.sampleGoalProb = .05       # Probability of sampling the goalPose
        self.plotEvery = 40             # Plot every this many new nodes
        self.closeEnoughEucl = .5
        self.closeEnoughAng = .5
        
        # Store parameters
        self.robot = robot.copy()
        self.polyMap = polyMap          # TODO: MAKE A MAP COPY
        self.gama = gama
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
        
        self.goalPose = None                # A 1x3 numpy array
        self.iterations = 0
        self.nodeList = []
        self.goalList = []
        
    def setCloseEnough(self, euclidean, angle):
        self.closeEnoughEucl = euclidean
        self.closeEnoughAng = angle        
        
    def runKIterations(self, fromPose, toPose, K):
        # Setup 
        startNode = self.Node.nodeFromPose(fromPose) 
        startNode.costFromParent = 0
        startNode.costFromRoot = 0
        
        self.nodeList = [startNode]
        self.goalList = []
        
        self.goalPose = toPose
        self.iterations = 0
        
        # Run the desired number of iterations
        for i in range(K):
            print "Iteration: ", i
            self.iterate()
            
#         # TODO: REMOVE
#         self.goalList.append(self.nodeList[-1])
            
        # Show results
        print "Num goals: ", len(self.goalList)
        print "Num nodes: ", len(self.nodeList)
        nodePath = self.getBestNodePathFromTree()
        if nodePath is None:
            print "No path found..."
            return
        if self.plotting:            
            self.plotter.clearPlot()
            self.plotter.drawMap(self.polyMap)
            self.plotter.drawTree(self.nodeList)
            
            trajectory = self.nodesToTrajectory(nodePath)
            self.plotter.drawPath2D(trajectory, color='g', width=2)
            self.plotter.drawStartAndGoalPoints(fromPose, toPose)
        
    def getBestNodePathFromTree(self):
        """ After growing the tree, use to check for best resulting path.
        Returns a list of Nodes
        """
        bestGoal = None
        bestDist = np.inf
        
        # Find the goal with the shortest distance from root
        for node in self.goalList:
            if node.costFromRoot < bestDist:
                bestGoal = node
                bestDist = node.costFromRoot
                
        # If we found the goal generate the path from it else return None
        if bestGoal is not None:
            path = [] 
            currNode = bestGoal
            while currNode is not None:
                path.append(currNode)
                currNode = currNode.parent
            path.reverse()
            return path
        else:
            return None
    
    def getBestDipolePathFromTree(self):
        """ After growing the tree, use to check for best resulting path.
        Returns a list of 1x3 arrays
        """
        nodePath = self.getBestNodePathFromTree()
        if nodePath is not None:
            return [node.getPose() for node in nodePath]
        else:
            return None
        
    def nodesToTrajectory(self, nodePath):
        paths = [self.getDipolarPath(nodePath[i], nodePath[i+1])
                 for i in range(len(nodePath) - 1)]
        
        # Concatenate all of the paths
        trajectory = []
        map(trajectory.extend, paths)
        return trajectory
            
    def sampleFree(self):
        if random() < self.sampleGoalProb:
            randPose = self.goalPose
        else:
            # TODO: Change the way in which map is used
            randPoint = self.polyMap.cFree.sample(random)
            randAng = random() * 2 * np.pi 
            randPose = (randPoint[0], randPoint[1], randAng)
            
        return self.Node.nodeFromPose(randPose)
    
    def drive(self, fromNode, newNode):
        """ Move newNode such that it is no further than self.stepSize from 
        fromNode. Return None if a collision exists.
        """
        dirVect = newNode.dirVectFrom(fromNode)
        distance = norm(dirVect)
        if distance > self.stepSize:
            dirVect = dirVect/distance * self.stepSize
            newNode.setPosition(fromNode.getPosition() + dirVect)
            
        isCollisionFree = self.polyMap.cFree.covers
        self.robot.moveTo(newNode.getPose())
        if isCollisionFree(self.robot.shape):
            return newNode
        else:
            return None
    
    def distance(self, fromNode, toNode):
        return norm(toNode.getPosition() - fromNode.getPosition())
    
    def closeEnough(self, currPose, desPose):
        diffAngle = abs((currPose[2] - desPose[2] + np.pi)%(2*np.pi) - np.pi)
        if (diffAngle < self.closeEnoughAng and 
            norm(desPose[:2] - currPose[:2]) < self.closeEnoughEucl):
            return True
        else:
            return False
    
    def sortByDistances(self, fromNodes, toNode):
        """ Returns the nodes oredered by distances along with the 
        corresponding distances.
        """
        numNodes = len(fromNodes)
        distances = map(self.distance, fromNodes, [toNode]*numNodes)
        nodeAndDist = zip(fromNodes, distances)
        sortedList = sorted(nodeAndDist, key=lambda x: x[1])
        nodes, distances = zip(*sortedList)
        return nodes, distances
    
    def nearest(self, fromNodes, toNode):
        numNodes = len(fromNodes)
        distances = map(self.distance, fromNodes, [toNode]*numNodes)
        minIndex = min(range(numNodes), key=lambda x: distances[x])
        return fromNodes[minIndex]
        
    def near(self, fromNodes, toNode):
        """ Returns the list of nodes that are within a radius of toNode.
        The radius is calculated using the formula give nin the RRT* paper.
        """
        numNodes = len(self.nodeList) 
        
        # gama * ( log(Card(V)) / Card(V) ) ^ (1/D)
        radius = self.gama * (np.log(numNodes)/numNodes)**(1/2)
        
        sortedNodes, distances = self.sortByDistances(self.nodeList, toNode)
        return [sortedNodes[i] for i in range(numNodes) if distances[i] < radius]
    
    def findBestCollisionFree(self, fromNodes, toNode):
        """ Find the best node to connect from such that there are no 
        collisions.
        """
        sortedNodes, _ = self.sortByDistances(fromNodes, toNode)
        for node in sortedNodes:
            if not self.hasCollision(node, toNode):
                return node
            
        return None
    
    def getDipolarPath(self, fromNode, toNode, timeout=.5):
        """ Returns the path traveled as a list of 1x3 arrays or None if a 
        collision or timeout occurs. 
        
        :param timeout: The maximum time to spend in this function
        """        
        poseCurr = fromNode.getPose()
        poseDes = toNode.getPose()
        posePrev = poseCurr
        path = [poseCurr]
        
        # Rename functions
        getControlls = self.controller.getControlls
        integrateForwards = self.controller.integrateForwards
        isCollisionFree = self.polyMap.cFree.covers
        DT = self.colCheckInter
        
        timeStart = clock()
        while (clock() - timeStart) < timeout:
            u, w = getControlls(poseCurr, poseDes, posePrev, DT)
            posePrev = poseCurr
            poseCurr = integrateForwards(posePrev, u, w, DT)
            path.append(poseCurr)
            
            # Check for collision
            self.robot.moveRobotTo(poseCurr)
            if not isCollisionFree(self.robot.shape):
                return None
            
            # If reached goal return the final state
            if self.closeEnough(poseCurr, poseDes):
                return path
            
        # Timed out. This warning can be removed if bothersome.
        print "DEBUG: Warning getDipoleToDipolePath timed out."
        
        return None
    
    def get2DPathLength(self, path):
        """ Get the length of a path along the set of 2D waypoints.
        
        :param path: A list of points or poses as numpy arrays.
        """
        distances = [norm(path[i][:2] - path[i+1][:2]) 
                     for i in range(len(path)-1)]
        return sum(distances)
    
    def cost(self, fromNode, toNode):
        path = self.getDipolarPath(fromNode, toNode)
        if path is not None:
            return self.get2DPathLength(path)
        else:
            return np.inf
    
    def hasCollision(self, fromNode, toNode):
        """ Check for collisions in intervals. 
        """
        if self.getDipolarPath(fromNode, toNode) is None:
            return True
        else:
            return False        
    
    def addNode(self, newNode, parent):
        """ Add a new node to the tree. Update its fields.
        """
        newNode.parent = parent
        newNode.costFromParent = self.distance(parent, newNode)
        newNode.costFromRoot = parent.costFromRoot + newNode.costFromParent
        self.nodeList.append(newNode)
    
    def rewire(self, newNode, nearNodes):
        """ Rewire paths to nearNodes if paths from newNode are shorter.
        """
        costBase = newNode.costFromRoot
        for node in nearNodes:
            costT = self.cost(newNode, node)
            newCost = costBase + costT
            if newCost < node.costFromRoot and not self.hasCollision(newNode, node):
                node.parent = newNode
                node.costFromParent = costT
                node.costFromRoot = newCost
                
    # TODO: REMOVE
    def testRewire(self):
        node1 = self.Node((0,0))
        node1.costFromParent = 0
        node1.costFromRoot = 0
        
        node2 = self.Node((10,10), parent=node1)
        node2.costFromParent = self.distance(node2.parent, node2)
        node2.costFromRoot = node2.costFromParent
        
        node3 = self.Node((10,0), parent=node2)
        node3.costFromParent = self.distance(node3.parent, node3)
        node3.costFromRoot = node3.parent.costFromRoot + node3.costFromParent
        
        node4 = self.Node((5,0), parent=node1)
        node4.costFromParent = self.distance(node4.parent, node4)
        node4.costFromRoot = node4.parent.costFromRoot + node4.costFromParent
        
        listT = [node1, node2, node3, node4]
        self.rewire(node4, listT)
    
    def iterate(self):
        """ Go through one iteration of the RRT* algorithm
        """
        self.iterations += 1
        
        # Sample a new state
        randNode = self.sampleFree()
        
        # Drive node closer to tree if necessary/possible
        closestNode = self.nearest(self.nodeList, randNode)
        newNode = self.drive(closestNode, randNode)
        if newNode is None:
            return
        
        # Get all near vertices
        nearNodes = self.near(self.nodeList, newNode)
        nearNodes.append(closestNode)
        
        # Find the best parent and update the newNode
        parent = self.findBestCollisionFree(nearNodes, newNode)
        if parent is None:
            return
        
        # Add trajectory from best parent
        self.addNode(newNode, parent)
        
        # Re-wire tree
        self.rewire(newNode, nearNodes)
            
        # Note if we have reached the goalPose
        # TODO: Change to allow multiple goals and smarter approach
        if self.closeEnough(newNode.getPose(), self.goalPose):
            self.goalList.append(newNode)
            
        # Plot
        if self.plotting and len(self.nodeList) % self.plotEvery == 0:
            self.plotter.clearPlot()
            self.plotter.drawMap(self.polyMap)
            self.plotter.drawTree(self.nodeList)
        if DEBUG:
#             self.plotter.pause(.01)
            print "Num neighbors: ", len(nearNodes)
    
    
    