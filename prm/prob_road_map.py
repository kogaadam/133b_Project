#!/usr/bin/env python3
#
#   prm.py
#
#   Probabilistic Roadmap Planner Framework
#
import bisect
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import numpy as np
import random
import argparse

#from planarutils import PointInTriangle
#from planarutils import SegmentCrossTriangle
#from planarutils import SegmentNearSegment


# **************************** MODIFIED START ****************************
parser = argparse.ArgumentParser()
parser.add_argument("--N", required=True, help="number of nodes")
parser.add_argument("--K", required=True, help="K")
args = parser.parse_args()
# ***************************** MODIFIED END *****************************

#
#   General World Definitions
#
#   List of objects, start, and goal
#
(xmin, xmax) = (0.1, 0.9)
(ymin, ymax) = (0.1, 0.9)
(zmin, zmax) = (0.1, 0.9)

(startx, starty, startz) = ( 0.2, 0.2, 0.2)
(goalx,  goaly, goalz)  = ( 0.5, 0.5, 0.5)

N  = int(args.N)  # Perhaps as low as 20?  I'd probably cap at 2000?
K  = int(args.K)     # 5-10 for 2D, a little more for 3D?
K2 = 2*K   # 2K seems to work for me.


#
#   Visualization Tools
#
class Visual():
    def StartFigure():
        # Clear the current, or create a new figure.
        plt.clf()

        # Create a new axes, enable the grid, and set axis limits.
        fig = plt.figure(num=1, clear=True)
        ax = fig.add_subplot(1, 1, 1, projection='3d')

        x = np.array([[0, 0], [1, 1]])
        y = np.array([[0, 0], [0, 0]])
        z = np.array([[0, 1], [0, 1]])

        ax.plot_surface(x, y, z, alpha=0.5)

        x = np.array([[0, 0], [0, 0]])
        y = np.array([[0, 0], [1, 1]])
        z = np.array([[0, 1], [0, 1]])

        ax.plot_surface(x, y, z, alpha=0.5)

        x = np.array([[1, 1], [1, 1]])
        y = np.array([[0, 1], [0, 1]])
        z = np.array([[0, 0], [1, 1]])

        ax.plot_surface(x, y, z, alpha=0.5)

        x = np.array([[0, 0], [1, 1]])
        y = np.array([[1, 1], [1, 1]])
        z = np.array([[0, 1], [0, 1]])

        ax.plot_surface(x, y, z, alpha=0.5)
        ax.set(xlabel='x', ylabel='y', zlabel='z')

        return ax 


    def FlushFigure():
        # Show the plot.
        plt.pause(0.001)
        
    def ShowFigure():
        # Flush the figure and wait.
        Visual.FlushFigure()
        #input("Hit return to continue")


    def DrawState(ax, s, *args, **kwargs):
        ax.scatter3D(s.x, s.y, s.z, *args, **kwargs)

    def DrawLocalPath(ax, head, tail, *args, **kwargs):
        ax.plot3D([head.x, tail.x], [head.y, tail.y], [head.z, tail.z], *args, **kwargs)


#
#   Object State
#
def AngleDiff(t1, t2):
    return (t1-t2) - math.pi * round((t1-t2)/math.pi)

class State:
    def __init__(self, x, y, z):
        # Remember the position.
        self.x = x
        self.y = y
        self.z = z

        # Pre-compute any other information you might want.
        pass

    # Compute/create an intermediate state.  This can be useful if you
    # need to check the local planner by testing intermediate states.
    def Intermediate(self, other, alpha):
        return State(self.x + alpha * (other.x-self.x),
                     self.y + alpha * (other.y-self.y), 
                     self.z + alpha * (other.z-self.z))

# **************************** MODIFIED START ****************************
    #### These are necessary for the PRM:
    # Check whether in free space.
    def InFreespace(self):
        return True

    # Compute the relative distance to another state.    
    def Distance(self, other):
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2)

    # Check the local planner - whether this connects to another state.
    def ConnectsTo(self, other):
        return True
# ***************************** MODIFIED END *****************************

#
#   PRM Graph and A* Search Tree
#
class Node(State):
    def __init__(self, *args):
        # Initialize the state
        super().__init__(*args)

        # List of neighbors (for the graph)
        self.neighbors = []

        # Parent and costs (for A* search tree)
        self.seen        = False
        self.done        = False
        self.parent      = []
        self.costToReach = 0
        self.costToGoEst = 0
        self.cost        = self.costToReach + self.costToGoEst

    # Define the "less-than" to enable sorting by cost in A*.
    def __lt__(self, other):
        return self.cost < other.cost

    # Estimate the cost to go to another node, for use in A*.
    def CostToGoEst(self, other):
        return self.Distance(other)


#
#   A* Planning Algorithm
#
def AStar(nodeList, start, goal):
    # Prepare the still empty *sorted* on-deck queue.
    onDeck = []

    # Clear the search tree (to keep track).
    for n in nodeList:
        n.seen = False
        n.done = False

    # Begin with the start state on-deck.
    start.done        = False
    start.seen        = True
    start.parent      = None
    start.costToReach = 0
    start.costToGoEst = start.CostToGoEst(goal)
    start.cost        = start.costToReach + start.costToGoEst
    bisect.insort(onDeck, start)

    # Continually expand/build the search tree.
    while True:
        # Make sure we still have something to look at!
        if not (len(onDeck) > 0):
            return []

        # Grab the next node (first on deck).
        n = onDeck.pop(0)

        # Check whether we have found the goal.
        if (n == goal):
            break

        # Add the children to the on-deck queue.
        for c in n.neighbors:
            # Skip if already done.
            if c.done:
                continue

            # Check the cost for the new path to reach the child.
            costToReach = n.costToReach + n.Distance(c)
            costToGoEst = c.CostToGoEst(goal)
            cost        = costToReach + costToGoEst

            # Check whether it is already seen (hence on-deck)
            if c.seen:
                # Skip if the previous cost was better!
                if c.cost <= cost:
                    continue
                # Else remove from the on-deck list (to keep sorted).
                onDeck.remove(c)

            # Add to the on-deck list in the correct order.
            c.seen        = True
            c.parent      = n
            c.costToReach = costToReach
            c.costToGoEst = costToGoEst
            c.cost        = cost
            bisect.insort(onDeck, c)

        # Declare the node done.
        n.done = True

    # Build the path.
    path = [goal]
    while path[0].parent is not None:
        path.insert(0, path[0].parent)

    # Return the path.
    return path

# **************************** MODIFIED START ****************************
#
#   Select the Nodes
#
def AddNodesToList(nodeList, N):
    # Add uniformly distributed samples
    while (N > 0):
        x = np.random.uniform(xmin, xmax)
        y = np.random.uniform(ymin, ymax)
        z = np.random.uniform(zmin, zmax)

        node = Node(x,y,z)

        if node.InFreespace():
            nodeList.append(node)
            N -= 1 
# ***************************** MODIFIED END *****************************

#
#   Brute-Force Nearest Neighbor
#
def NearestNodes(node, nodeList, K):
    # Create a sorted list of (distance, node) tuples.
    list = []

    # Process/add all nodes to the sorted list, except the original
    # node itself.  Continually cap the list at K elements.
    for n in nodeList:
        if n is not node:
            bisect.insort(list, (node.Distance(n), n))
            list = list[0:K]

    # Return only the near nodes.
    return [n for _,n in list]


# **************************** MODIFIED START ****************************
def ConnectNearestNeighbors(nodeList, K):
    # Clear any and all existing neighbors.
    for node in nodeList:
        node.neighbors = []

    # Process every node in the list for find (K) nearest neighbors.
    # Get the (K2) nearest nodes, as some might not connection.  Once
    # a match in found, be sure to create the connnection from both
    # sides, i.e. add each node to each other's neighbor list.
    for node in nodeList:
        count = K
        for nearnode in NearestNodes(node, nodeList, K2):
            if node.ConnectsTo(nearnode):
                if (not nearnode in node.neighbors) and (not node in nearnode.neighbors):
                    node.neighbors.append(nearnode)
                    nearnode.neighbors.append(node)

                    count -= 1

            if count == 0:
                break;
# ***************************** MODIFIED END *****************************

#
#  Post Process the Path
#
def PostProcess(path):
    i = 0
    while (i < len(path)-2):
        if path[i].ConnectsTo(path[i+2]):
            path.pop(i+1)
        else:
            i = i+1


#
#  Main Code
#
def main():
    # Create the figure.
    ax = Visual.StartFigure()
    Visual.FlushFigure()
    input("Test")

    # Create the start/goal nodes.
    startnode = Node(startx, starty, startz)
    goalnode  = Node(goalx,  goaly, goalz)

    # Show the start/goal states.
    Visual.DrawState(ax, startnode, 'ro')
    Visual.DrawState(ax, goalnode,  'ro')
    Visual.ShowFigure()



    # Create the list of sample points.
    nodeList = []
    AddNodesToList(nodeList, N)

    # Show the sample states.
    for node in nodeList:
        Visual.DrawState(ax, node, 'kx')
    Visual.ShowFigure()

    # Add the start/goal nodes.
    nodeList.append(startnode)
    nodeList.append(goalnode)

    # Connect to the nearest neighbors.
    ConnectNearestNeighbors(nodeList, K)

    # Show the neighbor connections.
    for node in nodeList:
        for neighbor in node.neighbors:
            Visual.DrawLocalPath(ax, node, neighbor, 'g-', linewidth=0.5)
    Visual.ShowFigure()


    # Run the A* planner.
    path = AStar(nodeList, startnode, goalnode)
    if not path:
        print("UNABLE TO FIND A PATH")
        input("Hit return to continue")
        return 

    # Show the path.
    for i in range(len(path)-1):
        Visual.DrawLocalPath(ax, path[i], path[i+1], 'r-', linewidth=1)
    Visual.FlushFigure()


    # Post Process the path.
    PostProcess(path)

    # Show the post-processed path.
    for i in range(len(path)-1):
        Visual.DrawLocalPath(ax, path[i], path[i+1], 'b-', linewidth=2)
    Visual.ShowFigure()
    input("Hit return to continue")


if __name__== "__main__":
    main()
