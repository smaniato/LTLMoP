
from __future__ import division

from random import random, sample
from numpy.linalg import norm
from numpy import array
from time import clock
from _DipolarCLoopControl import diffAngles
from math import atan2
from Queue import PriorityQueue
import logging

DEBUG = False
DEBUG_PLT = False       # Matplotlib workaround when using debugger
PLOT_TREE_FAIL = True   # Plot tree if it failed to find a path
    
# TODO: METHODS FOR GETTING TO GOAL REGION AND POINT
# TODO: Robot should have a "buffer zone" when entering regions or do shape padding
# TODO: IT IS WEIRD THAT MAP IS EDDITED (ROBOT OFFSET ADDITION) TALK TO SPYROS
# TODO: MAKE SEPARATE FILE FOR CLOSE ENOUGH, DIFFANGLES, ETC.
# TODO: INCORPORATE INFLUENCE GAIN
# TODO: ADD *ARGS TO PLOTTER
# TODO: UPDATE PLOTTER FOR NEW NODES ETC

class NodeXYT:
    def __init__(self, pose, parent=None):
        self.pose = tuple(pose)
        self.parent = parent

class PoseList:    
    """ A wrapper to provide sampling and checks for reaching the poses.
    """
    def __init__(self, poses, closeEnoughFun):
        self.poses = [tuple(p) for p in poses]
        self.closeEnoughFun = closeEnoughFun
        
    def samplePose(self):
        return sample(self.poses, 1)[0]
    
    def isValidPose(self, pose):
        """ Checks if the given pose is close enough to any of the 
        stored poses.
        """
        for p in self.poses:
            if self.closeEnoughFun(pose, p):
                return True
        return False

class DipolarRRT:            
    
    def __init__(self, robot, controller, stepSize=1, 
                 plotter=None, plotTree=True):
        """ An RRT path planner that takes into account orientation requirements.
        
        :param robot: An RRTRobot object
        :param controller: A DipolarController object
        :param stepSize: The max euclidean distance between nodes
        :param plotter: An RRTPlotter object if plotting is desired
        :param plotTree: A boolean deciding if to plot the tree progressively
        """   
        # Settings
        self.fwdIntegrTime = .05        # Interval for checking collisions (time)
        self.sampleGoalProb = .05       # Probability of sampling the goal
        self.stepSize = stepSize
        
        self.closeEucl = .25
        self.closeEucl2 = self.closeEucl * self.closeEucl
        self.closeAng = .5
        
        # Store parameters
        self.robot = robot.copy()
        self.controller = controller    # Dipolar controller 
        self.plotter = plotter
        self.plotTree = plotTree
        
        if plotTree and plotter is None:
            raise Exception("Asked to plot tree but no plotter was provided.") 
        
    def setCloseEnough(self, euclidean, angle):
        self.closeEucl = euclidean
        self.closeEucl2 = euclidean**2
        self.closeAng = angle        
        
    pass
    #===========================================================================
    # Path Calculation
    #===========================================================================
        
    def getNodePath(self, fromPose, rrtMap, goal, K=500, addOffset=True):
        """ Returns a path as a list of nodes.
        
        :param fromPose: A pose with (X, Y, Theta) 
        :param rrtMap: An RRTMap object
        :param goal: A list of goal poses or an RRTMap of goal regions.
        :param K: The number of iterations to perform
        :param addOffset: True if robot may not be fully inside of any region
            and this has not been handled anywhere else
        """
        tree = [NodeXYT(fromPose)]
        self.tree = tree
        if type(goal) is list:
            # Use a wrapper to sample
            goal = PoseList(goal, self.closeEnoughPose)
                             
        # In case robot is not fully inside of regions initially
        if addOffset: 
            self.robot.moveTo(fromPose)
            self.rrtMap = rrtMap.copy()
            self.rrtMap.addRobotBuffer(self.robot)
        
        # Iterate until a path is found
        for i in range(K):
            if DEBUG:
                print "Iteration: ", i
                
            prevLen = len(tree)
            
            # Sample a new state and try to extend
            randNode = self.sampleFree(goal)
            self.connectDip(tree, randNode)
            
            numNew = len(tree) - prevLen
            
            # Plot or debug
            if self.plotTree and numNew > 0:
                self.plotter.drawTree(tree[-numNew:])
                if DEBUG_PLT:
                    self.plotter.pause(.0001)
            
            # Check if goal was reached
            if numNew > 0:
                for node in tree[-numNew:]:
                    if goal.isValidPose(node.pose):
                        return self.getPathThroughParents(node)
        
        # Show failing tree
        if self.plotter is not None and PLOT_TREE_FAIL:
            self.plotter.drawTree(tree)
            self.plotter.ioff()
            print "Showing failed result..."
            self.plotter.show()
        
        return None        
        
    def applyDijkSmooth(self, path, addOffset=True):
        """ Use the shortcut Dijkstra to return a shorter path as a list 
        of Nodes.
        
        :param path: A list of Nodes
        :param addOffset: True if robot may not be fully inside of any region
            and this has not been handled anywhere else
        """
        
        # In case robot is not fully inside of regions initially
        if addOffset: 
            self.robot.moveTo(path[0].pose)
            rrtMapOrig = self.rrtMap
            rrtMapTemp = rrtMapOrig.copy()
            rrtMapTemp.addRobotBuffer(self.robot)
            self.rrtMap = rrtMapTemp
                        
        # Setup
        numNodesTotal = len(path)
        goalNode = path[-1]
        
        nodeIQueue = PriorityQueue()        # NodeXYT to expand (distance, nodeIndex)
        visited = [False]*numNodesTotal
        distances = {}                      # (nodeIndex:distanceFromStart)
        
        # Initialize 
        nodeIQueue.put((0,0))
        distances[0] = 0
        path[0].parent = None
        
        shortPath = None
        while not nodeIQueue.empty():
            currNodeI = nodeIQueue.get()[1]
            if visited[currNodeI]:
                continue
            currDist = distances[currNodeI]
            currNode = path[currNodeI]
            visited[currNodeI] = True
            
            # Check if reached Goal
            if currNode == goalNode:
                
                shortPath = []
                while currNode is not None:
                    shortPath.append(currNode)
                    currNode = currNode.parent
                shortPath.reverse()
                break
            
            # Go through all the non visited nodes
            for neighborI in range(numNodesTotal):
                if visited[neighborI]:
                    continue
                neighNode = path[neighborI]
                
                # Calculate alternative distance 
                pathT = self.getDipolarPath(currNode, neighNode)
                if pathT is None:
                    continue
                pathT += [neighNode.pose]
                distAlternative = currDist + self.get2DPathLengthE2(pathT)
                
                # If shorter than recorded update
                if distAlternative < distances.get(neighborI, float('inf')):
                    nodeIQueue.put((distAlternative, neighborI))
                    distances[neighborI] = distAlternative
                    neighNode.parent = currNode
                   
        if addOffset:
            self.rrtMap = rrtMapOrig
            
        return shortPath
        
    pass
    #===========================================================================
    # Node Manipulation
    #===========================================================================
        
    def nodesToPoses(self, nodeList):
        """ Returns a list of tuples with the poses of each node.
        """
        return [node.pose for node in nodeList]
    
    def nodesToTrajectory(self, nodePath, addOffset=True):
        """ Returns a list of tuples containing the poses of the robot as it
        travels the nodePath.  
        """
        # If robot is not completely inside of a region initially
        if addOffset: 
            self.robot.moveTo(nodePath[0].pose)
            rrtMapOrig = self.rrtMap
            rrtMapTemp = rrtMapOrig.copy()
            rrtMapTemp.addRobotBuffer(self.robot)
            self.rrtMap = rrtMapTemp
        
        getDP = self.getDipolarPath
        paths = [getDP(nodePath[i], nodePath[i+1])
                 for i in range(len(nodePath) - 1)]
        
        # Restore map
        if addOffset:
            self.rrtMap = rrtMapOrig
        
        # Concatenate all of the paths
        trajectory = []
        map(trajectory.extend, paths)
        trajectory.append(nodePath[-1].pose)
        return trajectory  
    
    pass
    #===========================================================================
    # Non-User Methods
    #===========================================================================
       
    def sampleFree(self, goal):
        if random() < self.sampleGoalProb:
            pose = goal.samplePose()
        else:
            pose = self.rrtMap.samplePose()
            
        return NodeXYT(pose)
     
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
        return self.distancePointE2(fromNode.pose[:2], toNode.pose[:2])
    
    def sortByDistancesE2(self, fromNodes, toNode):
        """ Returns the nodes ordered by distances along with the 
        corresponding distances.
        """
        numNodes = len(fromNodes)
        toNodeIter = (toNode for _ in range(numNodes))
        distances = map(self.distanceNodeE2, fromNodes, toNodeIter)
        nodeAndDist = zip(fromNodes, distances)
        sortedList = sorted(nodeAndDist, key=lambda x: x[1])
        nodes, distances = zip(*sortedList)
        return nodes, distances
    
    def closeEnoughNode(self, fromNode, toNode):
        return self.closeEnoughPose(fromNode.pose, toNode.pose)
               
    def closeEnoughPose(self, fromPose, toPose):
        ang = abs(diffAngles(fromPose[2], toPose[2]))
        dist2 = self.distancePointE2(fromPose[:2], toPose[:2])
        if (ang < self.closeAng and dist2 < self.closeEucl2):
            return True
        else:
            return False
             
    def getDipolarPath(self, fromNode, toNode, timeout=.5):
        """ Returns the path traveled as a list of (x, y, t) tuples or None if a 
        collision or timeout occurs. 
        """        
        poseCurr = fromNode.pose
        poseDes = toNode.pose
        posePrev = poseCurr
        path = [poseCurr]
        
        # Rename functions
        getControls = self.controller.getControls
        integrateForwards = self.controller.integrateForwards
        isCollisionFree = self.rrtMap.meetsRegionConstraints
        moveRobotTo = self.robot.moveRobotTo
        robot = self.robot
        DT = self.fwdIntegrTime
        
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
            
        if DEBUG:
            print "DEBUG: Warning getDipoleToDipolePath timed out."
        
        return None   
    
    def get2DPathLengthE2(self, path):
        """ Get the length of a path along the set of 2D waypoints. 
        Use euclidean distance squared.
        
        :param path: A list of points or poses.
        """        
        distances = [self.distancePointE2(path[i][:2], path[i+1][:2]) 
                     for i in range(len(path)-1)]
        return sum(distances)
    
    def hasCollision(self, fromNode, toNode):
        """ Check for collisions in intervals. 
        """
        if self.getDipolarPath(fromNode, toNode) is None:
            return True
        else:
            return False        
              
    def connectDip(self, fromNodes, toNode):
        """ Uses a version of the connect heuristic to extend the tree towards
        the toNode. 
        """
        NUM_CONSIDERED = 40
        
        # Sort the nodes to try to extend from.
        ordered = self.sortByDistancesE2(fromNodes, toNode)[0][:NUM_CONSIDERED]
        
        # Try to extend from nodes in order. Only try next if no progress.
        toPose = array(toNode.pose)
        initialNumNodes = len(fromNodes)
        for node in ordered:
            collided = False
            
            # Try to connect to dipole in intervals
            currentPose = array(node.pose)
            dirV = toPose[:2] - currentPose[:2]
            dirAng = atan2(dirV[1], dirV[0])
            dist = norm(dirV)
            numIntervals = int(dist/self.stepSize)
            dirV *= self.stepSize/dist
            
            currNode = node
            currP = currentPose[:2]
            for _ in range(numIntervals):
                currP += dirV
                nextNode = NodeXYT((currP[0], currP[1], dirAng), currNode)
                
                if self.hasCollision(currNode, nextNode):
                    collided = True
                    break
                
                fromNodes.append(nextNode)
                currNode = nextNode
                
            # Attempt to connectDip to toNode
            if not collided and not self.hasCollision(currNode, toNode):
                toNode.parent = currNode
                fromNodes.append(toNode)
                
            # Check if any progress was made
            if len(fromNodes) > initialNumNodes:
                return
    
    def getPathThroughParents(self, node):
        path = []
        while node is not None:
            path.append(node)
            node = node.parent
        path.reverse()
        return path
        
        
        
        
    
    
    