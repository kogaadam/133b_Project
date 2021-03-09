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
from scipy import interpolate
import numpy as np
import random
import argparse
import struct
import time

#from planarutils import PointInTriangle
#from planarutils import SegmentCrossTriangle
#from planarutils import SegmentNearSegment

# parser = argparse.ArgumentParser()
# parser.add_argument("--N", required=True, help="number of nodes")
# parser.add_argument("--K", required=True, help="K")
# args = parser.parse_args()

# N  = int(args.N)  # Perhaps as low as 20?  I'd probably cap at 2000?
# K  = int(args.K)     # 5-10 for 2D, a little more for 3D?
N = 100
K = 20
K2 = 2*K   # 2K seems to work for me.

#   Visualization Tools
#
class Visual():
    def PlotFourWalls(ax, xmin, xmax, ymin, ymax, zmin, zmax,  *args, **kwargs):
        x = np.array([[xmin, xmin], [xmin, xmin]])
        y = np.array([[ymin, ymax], [ymin, ymax]])
        z = np.array([[zmin, zmin], [zmax, zmax]])

        ax.plot_surface(x, y, z, *args, **kwargs)

        x = np.array([[xmax, xmax], [xmax, xmax]])
        y = np.array([[ymin, ymax], [ymin, ymax]])

        ax.plot_surface(x, y, z, *args, **kwargs)

        x = np.array([[xmin, xmax], [xmin, xmax]])
        y = np.array([[ymin, ymin], [ymin, ymin]])

        ax.plot_surface(x, y, z, *args, **kwargs)

        x = np.array([[xmin, xmax], [xmin, xmax]])
        y = np.array([[ymax, ymax], [ymax, ymax]])

        ax.plot_surface(x, y, z, *args, **kwargs)

    def StartFigure(outside_walls, obstacles):
        # Clear the current, or create a new figure.
        plt.clf()

        # Create a new axes, enable the grid, and set axis limits.
        fig = plt.figure(num=1, clear=True)
        ax = fig.add_subplot(1, 1, 1, projection='3d')
        ax.set(xlabel='x', ylabel='y', zlabel='z')

        ((xmin, xmax), (ymin, ymax), (zmin, zmax)) = outside_walls
        Visual.PlotFourWalls(ax, xmin, xmax, ymin, ymax, zmin, zmax, alpha=0.5, color="blue")

        plt.xlim(xmin, xmax)
        plt.ylim(ymin, ymax)
        ax.set_zlim(zmin, max(xmax,ymax,zmax))

        for obs in obstacles: 
            ((xmin, xmax), (ymin, ymax), (zmin, zmax)) = (obs.xlim, obs.ylim, obs.zlim)
            Visual.PlotFourWalls(ax, xmin, xmax, ymin, ymax, zmin, zmax, alpha=0.9, color="green")

            x = np.array([[xmin, xmin], [xmax, xmax]])
            y = np.array([[ymin, ymax], [ymin, ymax]])
            z = np.array([[zmax, zmax], [zmax, zmax]])

            ax.plot_surface(x, y, z, alpha=0.9, color="green")

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

    def DrawWaypointState(ax, p, *args, **kwargs):
        ax.scatter3D(p[0], p[1], p[2], *args, **kwargs)

    def DrawLocalWaypointPath(ax, head, tail, *args, **kwargs):
        ax.plot3D([head[0], tail[0]], [head[1], tail[1]], [head[2], tail[2]], *args, **kwargs)

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

    #### These are necessary for the PRM:
    # Check whether in free space.
    def InFreespace(self):
        for obs in obstacles: 
            if obs.InObstacle(0.5, (self.x,self.y,self.z)):
                return False

        return True

    # Compute the relative distance to another state.    
    def Distance(self, other):
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2)

    # Check the local planner - whether this connects to another state.
    def ConnectsTo(self, other):
        alphas = np.linspace(0, 1, num=100)

        for alpha in alphas:
            state = self.Intermediate(other, alpha) 
            if (not state.InFreespace()):
                return False 

        return True

class Obstacle: 
    def __init__(self, xlim, ylim, zlim):

        self.xlim = xlim
        self.ylim = ylim
        self.zlim = zlim

    def InObstacle(self, d, p):
        px = p[0]
        py = p[1]
        pz = p[2]

        xbound = px > (self.xlim[0]-d) and px < (self.xlim[1]+d)
        ybound = py > (self.ylim[0]-d) and py < (self.ylim[1]+d)
        zbound = pz > (self.zlim[0]-d) and pz < (self.zlim[1]+d)

        return xbound and ybound and zbound 

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

#
#  Post Process the Path
#
def Simplify(path):
    i = 0
    while (i < len(path)-2):
        if path[i].ConnectsTo(path[i+2]):
            path.pop(i+1)
        else:
            i = i+1

def distance(p_i, p_f): 
    return np.sqrt((p_i[0] - p_f[0])**2 + (p_i[1] - p_f[1])**2 + (p_i[2] - p_f[2])**2)

def find_midpoint(p_i, p_f): 
    return ((p_i[0] + p_f[0])/2, (p_i[1] + p_f[1])/2, (p_i[2] + p_f[2])/2)

#
#   Interpolate points on the path 
#
def Interpolate(x_fine, y_fine, z_fine, d):
    new_waypoints = []
    
    i = 0 
    while (i < len(x_fine)-1):
        curr_p = (x_fine[i], y_fine[i], z_fine[i])
        next_p = (x_fine[i+1], y_fine[i+1], z_fine[i+1])

        segment_length = distance(curr_p, next_p)
        num_points = segment_length/d

        delta_x = (next_p[0]-curr_p[0])/num_points
        delta_y = (next_p[1]-curr_p[1])/num_points
        delta_z = (next_p[2]-curr_p[2])/num_points

        for j in range(round(num_points)):
            new_waypoints.append((curr_p[0] + j*delta_x, curr_p[1] + j*delta_y, curr_p[2] + j*delta_z))

        i += 1

    new_waypoints.append(waypoints[-1])

    return new_waypoints

def write_to_file(data):
    file = open("path.bin","wb")
    s = struct.pack('d'*len(data),*data)
    file.write(s)

#
#   General World Definitions
#
#   List of objects, start, and goal

outside_walls = ((-2, 2), (-2, 2), (0, 0.5))

(xmin, xmax) = (-1.6, 1.6)
(ymin, ymax) = (-1.6, 1.6)
(zmin, zmax) = (0, 1)

(startx, starty, startz) = (-1.5, -1.5, 0.075)
(goalx,  goaly, goalz)  = (1.25, 1.25, 0.5)

obstacles = []
#obs1 = Obstacle((-0.5, 0.0), (-0.5, 0.0), (0, 0.5))
#obs2 = Obstacle((-1.5, 0.5), (0.5, 1.0), (0, 0.75))
#obs3 = Obstacle((0.5, 1.0), (-0.5, -1.0), (0, 2))
obs1 = Obstacle((-0.5, 2.0), (-0.5, -1.0), (0, 0.75))
obs2 = Obstacle((-2.0, 0.5), (0.5, 1.0), (0, 0.75))
obstacles.append(obs1)
obstacles.append(obs2)

#
#  Main Code
#
def main():
    # Create the figuree
    ax = Visual.StartFigure(outside_walls, obstacles)
    Visual.FlushFigure()
    # input("Test")
    time.sleep(1)

    # Create the start/goal nodes.
    startnode = Node(startx, starty, startz)
    goalnode  = Node(goalx,  goaly, goalz)

    # Create the list of sample points.
    nodeList = []
    AddNodesToList(nodeList, N)

    # Add the start/goal nodes.
    nodeList.append(startnode)
    nodeList.append(goalnode)

    # Connect to the nearest neighbors.
    ConnectNearestNeighbors(nodeList, K)

    # Run the A* planner.
    path = AStar( nodeList, startnode, goalnode)
    if not path:
        print("UNABLE TO FIND A PATH")
        input("Hit return to continue")
        return 

    # Post Process the path.
    Simplify(path)

    waypoints = []
    for node in path: 
        p = (node.x, node.y, node.z)
        waypoints.append(p)
        Visual.DrawWaypointState(ax, p, 'ro', s=4)
    Visual.ShowFigure()
    #print(waypoints)

    # Show the post-processed path.
    for i in range(len(waypoints)-1):
        Visual.DrawLocalWaypointPath(ax, waypoints[i], waypoints[i+1], 'b-', linewidth=0.8)
    Visual.ShowFigure()
    # input("Hit return to continue")
    time.sleep(1)

    x_sample = [x[0] for x in waypoints]
    y_sample = [x[1] for x in waypoints]
    z_sample = [x[2] for x in waypoints]
    print("SAMPLE")

    tck, u = interpolate.splprep([x_sample,y_sample,z_sample], s=2)
    x_knots, y_knots, z_knots = interpolate.splev(tck[0], tck)
    u_fine = np.linspace(0,1,1000)
    x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)

    ax.scatter3D(x_fine, y_fine, z_fine, 'g', linewidth=0.5)
    plt.show()
    # input("Hit return to continue")
    time.sleep(1)

    data = []
    length = len(x_fine)
    for i in range(length):
        data.append(x_fine[i])
        data.append(y_fine[i])
        data.append(z_fine[i])

    write_to_file(data)

    print(len(x_fine))
    print('{',end='')
    # for i in range(len(x_fine)): 
    for i in range(10): 
        print('{',end='')
        print("{},{},{}".format(x_fine[i], y_fine[i], z_fine[i]),end='')
        print('},')
    # input("Hit return to continue")
    time.sleep(1)
    return 0


if __name__== "__main__":
    main()
