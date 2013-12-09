"""
TODO:
    # Implement shortcut smoothing
    # Introduce non-circular robot
    # Introduce closest node measure
    ----------------------------
    # Improved closest node measure
    # Fix plotting issue (look at close enough)
    # Implement CONNECT heuristic
    # Sped up the control calculations
    # straight line
    # Fix obstacle collision issues
    # Change getControls method to remove arrays
    # Try to expand to N closest not just closest
    # Plot robot at start position
    # Improved path smoothing to account for closeEnoughDipole()
    -------------------------------
    # Implement regular RRT with connect heuristic in 2D
    # Create Dijkstra for the 2D path
    # Create a method that inputs dipoles into the 2D path
    # Craete a method that tries to smooth corners in the 2d path
    # Implement A* for smoothing
    # Allow 2D to dipole have multiple shifts
    --------------------------------
    Place robot within the full map
    Fix discrepancies with actual path traveled 
        calculated from closeEnoughDipole()
    Implement multiple goals
    Add new maps
    Create handler
    Make use of rrtPath calculations
    Improve smoothing speed
    Decide if backwards or not (work around)
    Handler option for backing up
    Consider squaring differences in closeness measure
    Change diff angle to be relative to angle1
    drawPathDipoleControl should be something like path to 2d 
    fix issue with DEBUGGER mode
    figure out when robot is within the next region
    explain what cFree is in RRTMap
    timming issues with 
    fix test after changes are made
    possible issues with different dT for LTLMoP and RRT
    fix oscilations due to map size, velocity, velocity drop as approach dipole
    map can now have holes. update any part that needs to be updated (draw map)
    
QUESTIONS:
    Does hardcoding a dT make sense?
    if current_reg == next_reg and not last: 
    ammount to move into region from transition face is 
        distIntoPoly = self.robot.backLen * 1.1 + self.closeEnoughDist
    raise error if no goal pose is found
    a lot of warnings for not having certain files: ompl, nxtMotors, etc
    Can I down sample the map size in simulation? Transf matrix did not 
        allow floats
    
"""

from __future__ import division

from Queue import PriorityQueue
from random import random, randint
from time import clock

import Polygon
from numpy.linalg import norm

from _DipolarCLoopControl import DipolarController
from _PlotRRT import RRTPlotter
import Polygon.Shapes as pShapes
from _RRTMapAndRobot import RRTMap, RRTRobot
import matplotlib.pyplot as plt
import numpy as np


def diffAngles(angle1, angle2):
    """ Returns difference between -pi and pi. Relative to angle2.
    """
    return (angle1 - angle2 + np.pi)%(2*np.pi) - np.pi
    

class DipolarRRT:
    
    def __init__(self, fullMap, robot, plotter=None, dipController=None):
        """ An RRT path planner that takes into account orientation 
        requirements.
        
        Note: Dipoles and poses are always numpy arrays with x, y, and theta.

        :param fullMap: A RRTMap object
        :param robot: A robotRRT object
        :param plotter: A RRTPlotter object. If plotting is desired
        :param dipController: A DipolarController object
        """
                
        # TODO: CHECK TO SEE WHICH SHOULD BE KEPT HERE
        # DEBUGING TOOLS
        self.DEBUGER = False        # Debugger is on
        self.PLOT_TREE = False      # Plot RRT tree as it is grown
        self.PLOT_TREE_FAIL = False # Plot RRT tree in case of timeout
        
        # Settings
        self.connectDist = 1       # Euclidean distance between nodes for CONNECT
        self.closeEnoughDist = .25
        self.closeEnoughAng = .5
        
        self.fullMap = fullMap
        self.robot = robot
        if dipController is None:
            self.dipControl = DipolarController(k1=.5, k2=1.5, lambdaValue=3)
        else:
            self.dipControl = dipController
        self.plotter = plotter
        
    def updateMap(self, newMap):
        self.fullMap = newMap
        
    def setCloseEnoughVals(self, dist=None, ang=None):
        """ Set the values that are checked when closeEnoughDipole is called. """
        if dist is not None:
            self.closeEnoughDist = dist
        if ang is not None:
            self.closeEnoughAng = ang
            
    def setConnectDist(self, dist):
        self.connectDist = dist
            
    def get2DPathLength(self, path):
        """ Get the length of a path along the set of 2D waypoints.
        
        :param path: A list of points or poses as numpy arrays.
        """
        distances = [norm(path[i][:2] - path[i+1][:2]) 
                     for i in range(len(path)-1)]
        return sum(distances)

    def sampleDipole(self):
        """ Returns a random dipole within the map's C-Free
        """
        randPoint = self.fullMap.cFree.sample(random)
        randAng = 2 * np.pi * random()
        return np.array([randPoint[0], randPoint[1], randAng])
        
    def getDipoleDistanceHeur(self, dipoleStart, dipoleEnd):
        """ Calculate a heuristic distance from dipoleStart to dipoleEnd. The
        distance is a linear combination of the euclidean distance between
        dipoles, orientation difference between dipoles, orientation difference
        between dipoleStart and the field at dipoleStart from dipoleEnd, and the
        how far infront or behind dipoleStart is from the line orthogonal to 
        to and passing through dipoleEnd. 
        """
        # Weights
        W_EUCL = 5              # Euclidian distance
        W_ORIENT = 1            # Difference between orientations
        W_FIELD = 1             # Difference between orientation and field
        W_FRONT_BEHIND = 0      # In front or behind
        
        dirV = dipoleStart[:2] - dipoleEnd[:2]
        euclD = norm(dirV)
        
        angleDiff = np.abs(diffAngles(dipoleEnd[2], dipoleStart[2]))
        
        dipoleField = self.dipControl.getFieldAt(dipoleStart, dipoleEnd)
        fieldAngle = np.arctan2(dipoleField[1], dipoleField[0])
        fieldDiff = np.abs(diffAngles(fieldAngle, dipoleStart[2]))
        
        dipoleT = np.array(dipoleStart[:2])/norm(dipoleStart[:2])
        relativePos = np.dot(dipoleT, dirV)
        
        totalDist = W_EUCL*euclD + W_ORIENT*angleDiff + \
                    W_FIELD*fieldDiff + W_FRONT_BEHIND*relativePos
        
        return totalDist
    
    def getNClosestIndicesDipole(self, dipole, tree, N=1):
        """ Uses the distance heuristic to return a list with the indices 
        of the N closest nodes within the tree to dipole.    
        """
        distances = [self.getDipoleDistanceHeur(dipole, nodeX.pose) 
                     for nodeX in tree]
        closestIndices = sorted(range(len(distances)), key=lambda i: distances[i])
        return closestIndices[:N]
    
    def getDipoleToDipolePath(self, dipoleStart, dipoleEnd, timeout = .5):
        """ Returns (connected, path) where connected is True if a collision
        free path was found, and path is the list of poses. 
        
        :param timeout: The maximum time to spend in this function
        """
        DT = .1         # The time interval for controls and forward integration
        
        poseCurr = np.array(dipoleStart)
        posePrev = poseCurr
        path = [poseCurr]
        
        timeStart = clock()
        while (clock() - timeStart) < timeout:
            u, w = self.dipControl.getControlls(poseCurr, dipoleEnd, posePrev, DT)
            posePrev = poseCurr
            poseCurr = self.dipControl.integrateForwards(posePrev, u, w, DT)
            path.append(poseCurr)
            
            # Check for collision
            self.robot.moveRobotTo(poseCurr)
            if not self.fullMap.cFree.covers(self.robot.shape):
                return (False, path)
            
            # If reached goal return the final state
            if self.closeEnoughDipole(poseCurr, dipoleEnd):
                return (True, path)
            
        # Timed out. This warning can be removed if bothersome.
        print "DEBUG: Warning getDipoleToDipolePath timed out."
        
        return (False, path)
    
    def closeEnoughDipole(self, currPose, endPose):
        """ Returns true if the euclidean and angular distances between currPose
        and endPose is below the set threshold.
        """        
        dist = norm(currPose[:2] - endPose[:2])
        angDiff = np.abs(diffAngles(currPose[2], endPose[2]))
        
        if dist < self.closeEnoughDist and angDiff < self.closeEnoughAng:
            return True
        else:
            return False
    
    # TODO: Method has not been used in a long time. Must be updated.
    def extendTreeDipole(self, tree, dipole):
        """ Tries to extend the tree in the direction of the sampled dipole by
        one leaf.
        """
        MAX_DIST = 1
        
        # Choose the dipole to extend from
        closestNodeIndex = self.getNClosestIndicesDipole(dipole, tree, N=1)[0]
        closestNode = tree[closestNodeIndex]
        
        dirV = dipole[:2] - closestNode.XY
        dist = norm(dirV)
        if dist > MAX_DIST:
            dipole = np.array(dipole)
            dipole[:2] = closestNode.XY + (dirV/dist)*MAX_DIST
        
        pathFound, _ = self.getDipoleToDipolePath(closestNode.pose, dipole)
        
        if pathFound:
            tree.append(self.Node(pose=dipole, parent=closestNode))
            
            return True
        else:
            return False
    
    def extendTreeDipoleConnect(self, tree, dipole):
        """ Finds the closest node in three to dipole and tries to extend from 
        that node towards dipole in intervals of MAX_DIST for as many 
        iterations as possible. Will consider the top NUM_CONSIDERED nodes to see
        if any will make progress and extend from one of them. New nodes are added
        to tree.
        """
        MAX_DIST = self.connectDist
        NUM_CONSIDERED = 5
        
        # Get the index of the closest nodes to dipole
        closestNodeIndex = self.getNClosestIndicesDipole(dipole, tree, 
                                                         N=NUM_CONSIDERED)
        
        # If no progress is made then try the next closest
        startTreeLen = len(tree)
        for i in closestNodeIndex:
            closestNode = tree[i]
        
            dirV = dipole[:2] - closestNode.XY
            dirAng = np.arctan2(dirV[1], dirV[0])
            dist = norm(dirV)
            numIntervals = int(dist/MAX_DIST)
            dirV = dirV/dist*MAX_DIST               # Normalize to step size
            
            # Try to connect to dipole in intervals
            collided = False
            parentT = closestNode
            dipoleCurr = np.array(parentT.pose)
            dipoleNext = np.array(dipoleCurr)
            dipoleNext[2] = dirAng      
            for _ in range(numIntervals):
                dipoleNext[:2] += dirV[:2]  
                
                connected, _ = self.getDipoleToDipolePath(dipoleCurr, dipoleNext)
                if not connected:
                    collided = True
                    break
                
                currNode = self.Node(pose=dipoleNext, parent=parentT)
                
                tree.append(currNode)
                parentT = currNode
                dipoleCurr = currNode.pose
            
            # Attempt to connect to the last node
            if not collided:
                connected, _ = self.getDipoleToDipolePath(dipoleCurr, dipole)
                if connected:
                    currNode = self.Node(pose=dipole, parent=parentT)
                    tree.append(currNode)
                    
            # If the tree has grown then progress was made. Dont go to next neighbors
            if len(tree) > startTreeLen:
                return
    
    def getRRTDipoleControlPath(self, startPose, goalPoseList, goalBias=.2):
        """ Returns a path from startPose to an goalPoseList in the form of a 
        list of Nodes. Uses a dipole controller to connect nodes in the RRT.
        """
        goalBias = .2           # Percentage of attempt to connect to goalPoseList
        MAX_ITTER = 700         # Maximum number of tree iterations
        
        startPose = np.array(startPose).astype(float)
        goalPoseList = [np.array(goalPose).astype(float) for goalPose in goalPoseList]
    
        tree = [self.Node(pose=startPose, parent=None)]
        
        oldTreeLength = 1
        
        for currIter in range(MAX_ITTER):
            
            # Select dipole to drive towards
            randNumber = random()
            if randNumber <= goalBias:
                randIndex = randint(0, len(goalPoseList)-1)
                randDipole = goalPoseList[randIndex]
            else:
                randDipole = self.sampleDipole()
            
            # Try to extend the tree
    #         extended = extendTreeDipole(tree, randDipole, fullMap)
            self.extendTreeDipoleConnect(tree, randDipole)
            
            numNewNodes = len(tree) - oldTreeLength
            oldTreeLength = len(tree)
            
            if self.PLOT_TREE and self.plotter != None:
#                 print "Tree: iteration: " + str(currIter)
                if numNewNodes > 0:
                    if self.DEBUGER:
                        self.plotter.drawMap(self.fullMap)
                        self.plotter.drawTree(tree, color='k', width=1)
                        plt.show()
                    else:
                        self.plotter.drawTree(tree[-numNewNodes:], color='k', 
                                              width=2)
                        for goalPose in goalPoseList:
                            self.plotter.drawStartAndGoalPoints(startPose, 
                                                                goalPose)
                    
            # Reached end
            finished = False
            for goalPose in goalPoseList:
                if self.closeEnoughDipole(tree[-1].pose, goalPose):
                    finished = True
                    break
            
            if finished:
                
                print "Tree total iterations: " + str(currIter)
                
                path = []
                currNode = tree[-1]
                while currNode != None:
                    path.append(currNode)
                    currNode = currNode.parent
                path.reverse()
                return path
        
        if self.PLOT_TREE_FAIL:
            plt.ioff()
            self.plotter.drawMap(self.fullMap)
            self.plotter.drawTree(tree, color='k', width=1)
            plt.show()
            
        return None
    
    def getShortcutPathDipole(self, path, additionalGoals=None):
        """ Use the shortcut heuristic to return a shorter path in the form of
        a list of Nodes. The final node could be the last node in path or one
        of the nodes from additionalGoals Implements Dijkstras to find new 
        connections in the given path.
        
        :param path: A list of Nodes
        :param additionalGoals: A list of Nodes
        """
        numEdges = 0                # Keep track of number of edges calculated
        
        # Get a list of unique goals
        if additionalGoals is not None:
            allGoalNodes = [node for node in additionalGoals]
        else:
            allGoalNodes = []
        duplicate = False
        nodeT = path[-1]
        for node in allGoalNodes:
            if norm(nodeT.pose - node.pose) == 0:
                duplicate = True
                break
        if not duplicate:
            allGoalNodes.append(nodeT)
        allNodes = [node for node in path[:-1]] + allGoalNodes
        
        # Setup
        numNodesTotal = len(allNodes)
        
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
            currNode = allNodes[currNodeI]
            visited[currNodeI] = True
            
            # Check if reached Goal
            if currNode in allGoalNodes:
                
                print "ShortcutDipole numEdges: " + str(numEdges)
                
                shortPath = []
                while currNode != None:
                    shortPath.append(currNode)
                    currNode = currNode.parent
                shortPath.reverse()
                return shortPath
            
            # Go through all the non visited nodes
            for neighborI in range(numNodesTotal):
                if visited[neighborI]:
                    continue
                neighNode = allNodes[neighborI]
                
                numEdges += 1
                
                # Calculate alternative distance 
                pathFound, pathDi = self.getDipoleToDipolePath(currNode.pose, 
                                                               neighNode.pose)
                if not pathFound:
                    continue
                pathDi += [neighNode.pose]
                distAlternative = currDist + self.get2DPathLength(pathDi)
                
                # If shorter than recorded update
                if distAlternative < distances.get(neighborI, float('inf')):
                    nodeIQueue.put((distAlternative, neighborI))
                    distances[neighborI] = distAlternative
                    neighNode.parent = currNode
                   
        # No path was found 
        return None 

    def getThetaStarPath(self, path, additionalGoals=None):
        """ Use the ThetaStar to return a shorter path in the form of
        a list of Nodes. The final node could be the last node in path or one
        of the nodes from additionalGoals.
        
        :param path: A list of Nodes
        :param additionalGoals: A list of Nodes
        """
        # Get a list of unique goals
        if additionalGoals is not None:
            allGoalNodes = [node for node in additionalGoals]
        else:
            allGoalNodes = []
        duplicate = False
        nodeT = path[-1]
        for node in allGoalNodes:
            if norm(nodeT.pose - node.pose) == 0:
                duplicate = True
                break
        if not duplicate:
            allGoalNodes.append(nodeT)
            
        # The path without counting the goal
        partialPath = [node for node in path[:-1]]
        
        newPath = partialPath[:2]
        _, pathDipoleT = self.getDipoleToDipolePath(newPath[0].pose, 
                                                    newPath[1].pose)
        lengthPrev = self.get2DPathLength(pathDipoleT)
        
        # Run theta star on the partialPath
        for node in partialPath[2:]:
            # Calculate distance to new node through previous
            _, pathDipoleT = self.getDipoleToDipolePath(newPath[-1].pose, 
                                                        node.pose)
            length1 = self.get2DPathLength(pathDipoleT)
            
            # Check if find a shorter path through the prevous node's parent
            pathFoundT, pathDipoleT = self.getDipoleToDipolePath(
                                                newPath[-2].pose, node.pose)
            
            # Could not connect to the previous node's parent
            if not pathFoundT:
                newPath.append(node)
                lengthPrev = length1
                continue
            
            length2 = self.get2DPathLength(pathDipoleT)
            
            # First path is shorter
            if (lengthPrev + length1) < length2:
                newPath.append(node)
                lengthPrev = length1
                continue
            
            # Second path is shorter
            newPath[-1] = node
            lengthPrev = length2
           
        # Run theta star for each of the goals
        endDistancesAndNodes = []
        for goal in allGoalNodes:
            # Calculate distance to goal node through previous
            _, pathDipoleT = self.getDipoleToDipolePath(newPath[-1].pose, 
                                                        goal.pose)
            length1 = self.get2DPathLength(pathDipoleT)
            
            # Check if find a shorter path through the prevous node's parent
            pathFoundT, pathDipoleT = self.getDipoleToDipolePath(
                                                newPath[-2].pose, node.pose)
            
            # Could not connect to the previous node's parent
            if not pathFoundT:
                endNodesT = [newPath[-1], goal]
                endDistancesAndNodes.append(lengthPrev + length1, endNodesT)
                continue
            
            length2 = self.get2DPathLength(pathDipoleT)
            
            # First path is shorter
            if (lengthPrev + length1) < length2:
                endNodesT = [newPath[-1], goal]
                endDistancesAndNodes.append(lengthPrev + length1, endNodesT)
                continue
            
            # Second path is shorter
            endDistancesAndNodes.append((length2, [goal]))
            
        closestGoalI = min(range(len(endDistancesAndNodes)), 
                           key=lambda x: endDistancesAndNodes[x][0])
        
        return newPath[:-1] + endDistancesAndNodes[closestGoalI][1]    

    def get2DPathRepresent(self, path):
        """ Takes a path as a list of nodes and runs the dipolar controler to
        return a close approximation of the path traveled as a list of dipoles
        """
        pathTraveled = [path[0].pose]
        for i in range(len(path)-1):
            _, pathDi = self.getDipoleToDipolePath(path[i].pose, path[i+1].pose)
            pathDi.append(path[i+1].pose)
            pathTraveled += pathDi
        return pathTraveled        
    
    def nodesToDipoles(self, path):
        return [node.pose for node in path]
    
    def dipolesToNodes(self, dipoleList):
        """ Takes in a list of dipoles and outputs a list of Nodes """
        return [self.Node(pose=dipole) for dipole in dipoleList]
        
    class Node:
    
        def __init__(self, x=0, y=0, theta=0, XY=None, pose=None, parent=None):
            """ An object to represent poses and connections amongst them. If 
            instantiated, the value of later parameters will override the 
            previous (ex. pose can override all) 
    
            :param x: The x coordinate
            :param y: The y coordinate
            :param theta: The orientation in radians
            :param XY: List with x and y 
            :param pose: List with x, y, and theta
            :param parent: The node from which it stems
            """
            self.x = x
            self.y = y
            self.theta = theta
            
            if XY != None:
                self.x = XY[0]
                self.y = XY[1]
            if pose != None:
                self.x = pose[0]
                self.y = pose[1]
                self.theta = pose[2]
    
            self.parent = parent
            self.XY = np.array([self.x, self.y])
            self.pose = np.array([self.x, self.y, self.theta])


class TestRRT:    
    
    def __init__(self):
        self.DEBUGER = False            # Debugger is on
        self.PLOT_TREE = False           # Live tree
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

    def runRRTDipoleControlAndShortcut(self):
        """ Run the dipolar RRT/shortcut and plot according to parameters 
        specified in this class' init function.
        """
        if not self.DEBUGER:
            plt.ion()
            
        startPose, endPose, testMap = self.getSampleMapRRT(3)
        
        robotOutline = Polygon.Polygon([(-.25,0), (.25,0), (0,.5)])
        robot = RRTRobot(np.array([0,.2,np.pi/2]), robotOutline)
        
        figure, axes = plt.subplots()
        plotter = RRTPlotter(figure, axes)
        plotter.drawMap(testMap)
        axes.grid()
        
        plotter.drawStartAndGoalRobotShape(startPose, endPose, robot)
        
        planner = DipolarRRT(testMap, robot, plotter)
        planner.DEBUGER = self.DEBUGER
        planner.PLOT_TREE = self.PLOT_TREE
        planner.PLOT_TREE_FAIL = self.PLOT_TREE_FAIL
        
        rrtPath = planner.getRRTDipoleControlPath(startPose, endPose)
        
        if rrtPath == None:
            print "Did not find path in given time."
            return
        
        if self.PLOT_RRT_PATH:
            pathT = planner.get2DPathRepresent(rrtPath)
            plotter.drawDipolePath2D(pathT, color='g', width=3)
            
        print "Smoothing Path"
        shortPath = planner.getShortcutPathDipole(rrtPath)
        print "Done Smoothing Path"
        
        if self.PLOT_SHORT_PATH:
            pathT = planner.get2DPathRepresent(shortPath)
            plotter.drawDipolePath2D(pathT, color='r', width=3)
            
        print "Showing Final Results"
        plt.ioff()
        plt.show()
        
    def runRRTDipoleControlAndThetaStar(self):
        """ Run the dipolar RRT/shortcut and plot according to parameters 
        specified in this class' init function.
        """
        if not self.DEBUGER:
            plt.ion()
            
        startPose, endPose, testMap = self.getSampleMapRRT(3)
        
        robotOutline = Polygon.Polygon([(-.25,0), (.25,0), (0,.5)])
        robot = RRTRobot(np.array([0,.2,np.pi/2]), robotOutline, .5)
        
        plotter = RRTPlotter()
        plotter.drawMap(testMap)
        
        plotter.drawStartAndGoalRobotShape(startPose, endPose, robot)
        
        planner = DipolarRRT(testMap, robot, plotter)
        planner.DEBUGER = self.DEBUGER
        planner.PLOT_TREE = self.PLOT_TREE
        planner.PLOT_TREE_FAIL = self.PLOT_TREE_FAIL
        
        rrtPath = planner.getRRTDipoleControlPath(startPose, [endPose])
        
        if rrtPath == None:
            print "Did not find path in given time."
            return
        
        if self.PLOT_RRT_PATH:
            pathT = planner.get2DPathRepresent(rrtPath)
            plotter.drawDipolePath2D(pathT, color='g', width=3)
            
        print "Getting theta star Path"
        shortPath = planner.getThetaStarPath(rrtPath)
        print "Done Smoothing Path"
        
        if self.PLOT_SHORT_PATH:
            pathT = planner.get2DPathRepresent(shortPath)
            plotter.drawDipolePath2D(pathT, color='r', width=3)
            
        print "Showing Final Results"
        plt.ioff()
        plt.show()
        print 

    def runRRTDipoleControlAndShortcutNoPlot(self):     
        """ Use this method to time the implementation and/or profile it
        """       
        startPose, endPose, testMap = self.getSampleMapRRT(3)
        
        robotOutline = Polygon.Polygon([(-.25,0), (.25,0), (0,.5)])
        robot = RRTRobot(np.array([0,.2,np.pi/2]), robotOutline)
        
        planner = DipolarRRT(testMap, robot)
        planner.DEBUGER = False
        planner.PLOT_TREE = False
        planner.PLOT_TREE_FAIL = False
        
        rrtPath = planner.getRRTDipoleControlPath(startPose, endPose)
        
        if rrtPath == None:
            print "Did not find path in given time."
            return
            
        planner.getShortcutPathDipole(rrtPath)
        
pass
if __name__ == "__main__":
    print "Starting"
    timeS = clock()
       
    test = TestRRT()
#     test.runRRTDipoleControlAndShortcut()    
    test.runRRTDipoleControlAndThetaStar()
#     test.runRRTDipoleControlAndShortcutNoPlot()    
#     cProfile.run("test.runRRTDipoleControlAndShortcutNoPlot()") 
      
    print "Done: " + str(clock()-timeS)



