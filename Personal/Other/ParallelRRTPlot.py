
import sys
import matplotlib.pyplot as plt
import numpy as np

class Node:
    def __init__(self, idNum, x, y, parent):
        self.id = idNum
        self.x = x
        self.y = y
        self.parent = parent
        
class Polygon:
    def __init__(self):
        self.contours = []

def textLinesToNodes(textLines):
    def f(l):
        l = l.strip()
        parts = l.split(',')
        idNum, x, y = [float(v) for v in parts[:3]]
        if "NULL" in parts[3]:
            parent = None
        else:
            parent = int(parts[3])
        return Node(idNum, x, y, parent)
    return [f(l) for l in textLines]

def drawTree(nodeList):
    for node in nodeList:
        if node.parent is not None:
            parent = nodeList[node.parent]
            plt.plot([node.x, parent.x], [node.y, parent.y], 'k')       
            
def loadMap(mapfileName):
    with open(mapfileName, "r") as f:
        lines = f.readlines()
        
    mapPoly = Polygon()
        
    resetLine = None
    numVertices = None
    isHole = None
    vertices = []
    for i in range(1, len(lines)):        
        l = lines[i]
        if numVertices is None:
            numVertices = int(l)
            resetLine = i + 1 + numVertices
        elif isHole is None:
            isHole = bool(int(l))
        else:
            vertices.append([float(x) for x in l.split()])
        
        if i == resetLine:
            mapPoly.contours.append(vertices)
            
            numVertices = None
            isHole = None
            vertices = []
            
    return mapPoly
    
            
def drawPolygon(poly):
    for contour in poly.contours:
        vertices = np.array(contour + [contour[0]])
        plt.plot(vertices[:,0], vertices[:,1], color='k', linewidth=3)

def plotToFrame(inputName, outputName, numFrames, mapfileName):
    mapPoly = loadMap(mapfileName)
    
    with open(inputName, "r") as f:
        lines = f.readlines()[2:]
    
    # Start and goal
    _, startX, startY = lines[0].split()
    startX, startY = float(startX), float(startY)
    _, goalX, goalY = lines[1].split()
    goalX, goalY = float(goalX), float(goalY)
    
    nodeList = textLinesToNodes(lines[3:numFrames])
    
    drawPolygon(mapPoly)
    plt.plot([startX], [startY], "-ro")
    plt.plot([goalX], [goalY], "-go")
    drawTree(nodeList)
    
    plt.savefig(outputName, bbox_inces="tight")

    
if __name__ == "__main__":
    if len(sys.argv) == 1:
        print "Using default rrt.txt, rrt.pdf, 999999, map.txt"
        plotToFrame("rrt.txt", "rrt.pdf", 999999, "map.txt")
        
    elif len(sys.argv) == 5:
        inputName, outputName, numFrames, mapfileName = sys.argv[1:]
        numFrames = int(numFrames)
        plotToFrame(inputName, outputName, numFrames, mapfileName)
    
    else:
        print "Requires an inputfile.txt, outputfile.pdf, numberofframes, mapfile.txt"
        exit(0)  
        
    print "Done"
    
    