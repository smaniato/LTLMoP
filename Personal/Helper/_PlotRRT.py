#!/usr/bin/env python
"""
==================================================
PlotRRT
==================================================

This class uses matplotlib to aid in plotting several components of
the DipolarRRT algorithm.
"""

import numpy as np
import matplotlib.pyplot as plt
import time

class RRTPlotter:
    
    def __init__(self, figure=None, axes=None, invertY=False):
        """ An object to facilitate plotting common structures on the given 
        matplotlib figure and axes.
        """
        if figure is None or axes is None:
            self.fig, self.ax = plt.subplots()
        else:
            self.fig = figure
        self.invertY = invertY
#         self.ax.set_aspect('equal')
            
    def clearPlot(self):
        """ Clear the current axes. """
        plt.close()
        self.fig, self.ax = plt.subplots()
        
    def pause(self, t):
        """ Used when debugging. """
        plt.pause(t)
        
    def ion(self):
        """ Interactive on. """
        plt.ion()

    def ioff(self):
        """ Interactive off. """
        plt.ioff()

    def drawPolygon(self, poly, color='k', width=1, draw=True):
        """ Draw the outline of a polygon object
        """
        for contour in poly:
            vertices = np.array(contour + [contour[0]])
            if self.invertY:
                self.ax.plot(vertices[:,0], -vertices[:,1], 
                         color=color, linewidth=width)
            else:
                self.ax.plot(vertices[:,0], vertices[:,1], 
                         color=color, linewidth=width)
        if draw:
            plt.draw()
        
    def drawTree(self, tree, color='k', width=1, draw=True):
        """ For drawing trees created in RRT
        """
        for node in tree:
            parent = node.parent
            if parent != None:
                self.drawEdge(node, parent, color=color, width=width, draw=False)
        if draw:
            plt.draw()
                
    def drawEdge(self, node1, node2, color='k', width=1, draw=True):
        """ Draw the edge that connects two nodes from the RRT
        """
        n1x, n1y = node1.getPosition()
        n2x, n2y = node2.getPosition()
        
        if self.invertY:
            self.ax.plot([n1x, n2x], [-n1y, -n2y], color=color, linewidth=width)
#             self.ax.plot([n1x, n2x], [-n1y, -n2y], 
#                      'o-', color=color, linewidth=width)
        else:
            self.ax.plot([n1x, n2x], [n1y, n2y], color=color, linewidth=width)
#             self.ax.plot([n1x, n2x], [n1y, n2y], 
#                      'o-', color=color, linewidth=width)
        if draw:
            plt.draw()
    
    def drawStartAndGoalPoints(self, startPoint, goalPoint):
        """ Places a green marker at the start point and a red one at the end
        point
        
        :param startPoint: numpy 2D array
        :param gialPoint: numpy 2D array
        """
        if self.invertY:
            self.ax.plot(startPoint[0], -startPoint[1], 'go')
            self.ax.plot(goalPoint[0], -goalPoint[1], 'ro')            
        else:
            self.ax.plot(startPoint[0], startPoint[1], 'go')
            self.ax.plot(goalPoint[0], goalPoint[1], 'ro')
        plt.draw()
            
    def drawStartAndGoalRobotShape(self, startPose, goalPose, robot, draw=True):
        """ Plot the robots outline at the start and end pose
        
        :param startPose: numpy 3D array
        :param endPose: numpy 3D array
        :param robot: a RRTRobot object
        """
        robotTemp = robot.copy()
        
        robotTemp.moveRobotTo(startPose)   
        self.drawPolygon(robotTemp.shape, color='g', width=2, draw=False)
        
        robotTemp.moveRobotTo(goalPose)   
        self.drawPolygon(robotTemp.shape, color='r', width=2, draw=False)
        
        if draw:
            plt.draw()

    def drawMap(self, fullMap, draw=True):
        """ Draw the map with boundary and obstacles
        
        :param fullMap: a RRTMap object
        """
        self.drawPolygon(fullMap.boundary, color='k', width=3, draw=False)
         
        # Obstacles
        for obst in fullMap.allObstacles:
            self.drawPolygon(obst, color='b', width=3, draw=False)
        
        if draw:
            plt.draw()

    def drawMapConst(self, polyMap, draw=True):
        """ Draw the map with boundary and obstacles
        
        :param polyMap: a RRTMapConst object
        """
        for poly, _ in polyMap.regions:
            self.drawPolygon(poly, color='k', width=3, draw=False)
        
        if draw:
            plt.draw()
        
    def drawNodePath2D(self, path, color='k', width=1):
        """ Draw the path assuming that nodes are connected by straight lines
        
        :param path: a list of nodes
        """
        points = [node.getPosition() for node in path]
        self.drawPath2D(points, color=color, width=width)
    
    def drawPath2D(self, path, color='k', width=1, draw=True):
        """ Draw the path assuming that points are connected by straight lines
        
        :param path: a list 1x2 arrays
        """
        points = np.array(path)
        if self.invertY:
            self.ax.plot(points[:,0], -points[:,1], color=color, linewidth=width)
        else:
            self.ax.plot(points[:,0], points[:,1], color=color, linewidth=width)
        if draw:
            plt.draw()
    drawDipolePath2D = drawPath2D   # Depricated
        
    def drawRobotOccupiedPath(self, path, robot, color='k', width=1):
        """ Draw the space that the robot would occupy at each point in the
        path.
        
        :param path: a list of dipoles
        :param robot: an RRTRobot
        """
        robotCopy = robot.copy()
        for pose in path:
            robotCopy.moveRobotTo(pose)            
            self.drawPolygon(robotCopy.shape, color=color, width=width)
            time.sleep(1)
            
    def showMapAndTrees(self, fullMap, allTrees):
        """ Draw and plot the fullMap and trees. Mainly used to help debug
        
        :param fullMap: a RRTMap object
        :param allTrees: a list of trees from the RRT
        """
        self.drawMap(fullMap)
        
        for tree in allTrees:
            self.drawTree(tree, color='k', width=1)
            
        plt.show()
