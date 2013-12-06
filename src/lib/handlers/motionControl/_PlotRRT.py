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

class RRTPlotter:
    
    def __init__(self, figure=None, axes=None, invertY=False):
        """ An object to facilitate plotting common structures on the given 
        matplotlib figure and axes.
        """
        if figure is None or axes is None:
            self.fig, self.ax = plt.subplots()
        else:
            self.fig = figure
            self.ax = axes
        self.invertY = invertY
            
    def clearPlot(self):
        """ Clear the current axes. """
        plt.close()
        self.fig, self.ax = plt.subplots()
        
    def ion(self):
        """ Interactive on. """
        plt.ion()

    def ioff(self):
        """ Interactive off. """
        plt.ioff()

    def drawPolygon(self, poly, color='k', width=1):
        """ Draw the outline of a polygon object
        """
        vertices = np.array(poly[0] + [poly[0][0]])
        if self.invertY:
            self.ax.plot(vertices[:,0], -vertices[:,1], 
                     color=color, linewidth=width)
        else:
            self.ax.plot(vertices[:,0], vertices[:,1], 
                     color=color, linewidth=width)
        plt.draw()
        
    def drawTree(self, tree, color='k', width=1):
        """ For drawing trees created in RRT
        """
        for node in tree:
            parent = node.parent
            if parent != None:
                self.drawEdge(node, parent, color, width)
                
    def drawEdge(self, node1, node2, color='k', width=1):
        """ Draw the edge that connects two nodes from the RRT
        """
        if self.invertY:
            self.ax.plot([node1.x, node2.x], [-node1.y, -node2.y], 
                     'o-', color=color, linewidth=width)
        else:
            self.ax.plot([node1.x, node2.x], [node1.y, node2.y], 
                     'o-', color=color, linewidth=width)
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
            
    def drawStartAndGoalRobotShape(self, startPose, goalPose, robot):
        """ Plot the robots outline at the start and end pose
        
        :param startPose: numpy 3D array
        :param endPose: numpy 3D array
        :param robot: a RRTRobot object
        """
        robotTemp = robot.copy()
        
        robotTemp.moveRobotTo(startPose)   
        self.drawPolygon(robotTemp.shape, color='g', width=2)
        
        robotTemp.moveRobotTo(goalPose)   
        self.drawPolygon(robotTemp.shape, color='r', width=2)

    def drawMap(self, fullMap):
        """ Draw the map with boundary and obstacles
        
        :param fullMap: a RRTMap object
        """
        self.drawPolygon(fullMap.boundary, color='k', width=3)
         
        # Obstacles
        for obst in fullMap.allObstacles:
            self.drawPolygon(obst, color='b', width=3)
        
        plt.draw()
        
    def drawNodePath2D(self, path, color='k', width=1):
        """ Draw the path assuming that nodes are connected by straight lines
        
        :param path: a list of nodes
        """
        points = [node.XY for node in path]
        self.drawDipolePath2D(points, color=color, width=width)
    
    def drawDipolePath2D(self, path, color='k', width=1):
        """ Draw the path assuming that dipoles are connected by straight lines
        
        :param path: a list of dipoles
        """
        points = np.array(path)
        if self.invertY:
            self.ax.plot(points[:,0], -points[:,1], color=color, linewidth=width)
        else:
            self.ax.plot(points[:,0], points[:,1], color=color, linewidth=width)
        plt.draw()
            
    def showMapAndTrees(self, fullMap, allTrees):
        """ Draw and plot the fullMap and trees. Mainly used to help debug
        
        :param fullMap: a RRTMap object
        :param allTrees: a list of trees from the RRT
        """
        self.drawMap(fullMap)
        
        for tree in allTrees:
            self.drawTree(tree, color='k', width=1)
            
        plt.show()
