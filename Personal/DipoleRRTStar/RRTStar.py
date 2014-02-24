
from __future__ import division

from random import random
import numpy as np
from numpy.linalg import norm

DEBUG = False

# TODO: Find a good KDTree implementation
# from scipy.spatial import KDTree as Tree

import _RRTMapAndRobot
try:
    import _PlotRRT
    plotter_avail = True
except:
    print "Plotter disabled."
    plotter_avail = False
    

# TODO: dirVect = dirVect/distance * self.stepSize -> possible division by 0


class RRTStar:
    
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
        def nodeFromPose(cls, pose, parent):
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
            
    
    def __init__(self, robot, polyMap, gama=1.5, plotter=None):
        
        # Settings
        self.colCheckInter = .05        # Interval for checking collisions
        self.stepSize = 2               # Step size for growing tree
        self.sampleGoalProb = .1        # Probability of sampling the goalPose
        self.plotEvery = 40             # Plot every this many new nodes
        
        # Store parameters
        self.robot = robot.copy()
        self.polyMap = polyMap          # TODO: MAKE A MAP COPY
        self.gama = gama
        self.plotter = plotter
        
        self.goalPose = None                # A 1x2 numpy array
        self.iterations = 0
        self.nodeList = []
        self.goalList = []
        
    def runKIterations(self, fromPose, toPose, K):
        # Setup 
        startNode = self.Node(fromPose)
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
            
        # Plot resulting tree
        self.plotter.clearPlot()
        self.plotter.drawMap(self.polyMap)
        self.plotter.drawTree(self.nodeList)
        self.plotter.drawStartAndGoalPoints(fromPose, toPose)
        
    def getBestDipolePathFromTree(self):
        """ After growing the tree, use to check for best resulting path.
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
            while currNode.parent is not None:
                path.append(currNode)
                currNode = currNode.parent
            path.reverse()
            return [node.getPosition() for node in path]
        else:
            return None
            
    def sampleFree(self):
        if random() < self.sampleGoalProb:
            randPoint = self.goalPose
        else:
            # TODO: Change the way in which map is used
            randPoint = self.polyMap.cFree.sample(random)
            
        return self.Node(randPoint)
    
    def drive(self, fromNode, newNode):
        """ Move newNode such that it is no further than self.stepSize from 
        fromNode. Return None if a collision exists.
        """
        dirVect = newNode.dirVectFrom(fromNode)
        distance = norm(dirVect)
        if distance > self.stepSize:
            dirVect = dirVect/distance * self.stepSize
            newNode.setPosition(fromNode.getPosition() + dirVect)
            
        if self.hasCollision(fromNode, newNode):
            return None
        else:
            return newNode
        
    
    def distance(self, fromNode, toNode):
        return norm(toNode.getPosition() - fromNode.getPosition())
    
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
    
    def hasCollision(self, fromNode, toNode):
        """ Check for collisions in intervals. 
        """
        dirVect = toNode.getPosition() - fromNode.getPosition()
        
        distance = norm(dirVect)
        steps = int(distance/self.colCheckInter)
        
        # Check for corner case. Nodes are in same position.
        if distance > 0:
            dirVect = dirVect/distance * self.colCheckInter
        
            # Move by interval and use polygons for collision checks
            currPose = fromNode.getPosition()
            for _ in range(steps):            
                currPose += dirVect
                self.robot.moveRobotToPosition(currPose)
                if not self.polyMap.cFree.covers(self.robot.shape):
                    return True
            
        self.robot.moveRobotToPosition(toNode.getPosition())
        if not self.polyMap.cFree.covers(self.robot.shape):
                return True
        
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
        costT = newNode.costFromRoot
        for node in nearNodes:
            distT = self.distance(newNode, node)
            newCost = distT + costT
            if newCost < node.costFromRoot and not self.hasCollision(newNode, node):
                node.parent = newNode
                node.costFromParent = distT
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
        if nearNodes is not None:
            self.rewire(newNode, nearNodes)
            
        # Note if we have reached the goalPose
        # TODO: Change to allow multiple goals and smarter approach
        if norm(newNode.getPosition() - self.goalPose) == 0:
            self.goalList.append(newNode)
            
        # Plot
        if len(self.nodeList) % self.plotEvery == 0:
            self.plotter.clearPlot()
            self.plotter.drawMap(self.polyMap)
            self.plotter.drawTree(self.nodeList)
        if DEBUG:
#             self.plotter.pause(.01)
            print "Num neighbors: ", len(nearNodes)
    
    
    