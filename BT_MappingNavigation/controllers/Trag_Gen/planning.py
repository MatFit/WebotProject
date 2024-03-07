from asyncio.windows_events import NULL
import numpy as np
from matplotlib import pyplot as plt
from heapq import heapify, heappop, heappush
from collections import defaultdict
# from skimage.draw import line_nd, random_shapes
import random
import math
import py_trees
from threading import Thread
import multiprocessing


def world2map(xw,yw):
   px = int ((xw+2.25)*40)
   py = int ((yw-2)*(-50))

   px = min(px,199)
   py = min(px,299)
   px = max(px,0)
   py = max(py,0)

   return [px,py]

def map2world(px,py):
   xw = px/40-2.25
   yw=py/(-50)+2
   return [xw,yw]



# def world2map(xw, yw):
#     px = max(0, min(449, 449 * (xw + 2.26) / 4.5))
#     py = max(0, min(549, 549 * (1.832 - yw) / 5.667))
#     return px, py
# def map2world(px, py):
#     xw = (4.5 * px / 449) - 2.26
#     yw = 1.832 - (5.667 * py / 549)
#     return xw, yw

#Create Shortest Path method      
def getShortestPath(map, start, goal):
    def getNeighbors(v):
        neighbors = []
        for delta in ((0,1),(0,-1),(1,0),(-1,0),(1,1),(-1,1),(-1,1),(1,-1)):
            u = (delta[0] + v[0], delta[1] + v[1])
            if (u[0]>=0 and u[0]<len(map) and u[1]>=0 and u[1]<len(map[0]) and map[u[0]][u[1]]<0.3):
                neighbors.append((u, np.sqrt(delta[0]**2+delta[1]**2)))
        return (neighbors)
    
    queue = [(0, start)]
    heapify(queue)    

    distances = defaultdict(lambda: float('inf'))
    visited = set()
    parent = {}
    distances[start] = 0

    while queue:
        (currentdist, u) = heappop(queue)
        visited.add(u)
        if u == goal:
            queue.clear()
            break
        for (v, costuv) in getNeighbors(u):
            if v not in visited:
                new_dist = distances[u] + costuv
                if new_dist < distances[v]:
                    distances[v] = new_dist
                    parent[v] = u
                    heappush(queue, (distances[v]+np.sqrt((goal[0] - v[0]) ** 2 + (goal[1] - v[1]) ** 2), v))
    c = goal
    path = []
    while c in parent.keys():
        path.insert(0, c)
        c = parent[c]    
    return path
   



    
   
class Planning(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, goal):
        super(Planning, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard        
        px, py = world2map(goal[0], goal[1])
        self.goal = (px, py)
        self.end = goal[0],goal[1]
        self.WP = []

    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.display = self.robot.getDevice('display')  
    # Load Map
    def initialise(self):
        self.map = np.zeros((200,300))
        self.map = np.load('cspace.npy')
        self.display.setColor(0xFFFFFF)      
        for x in range(200):
            for y in range(300):
                if ((self.map[x][y])>0.9):
                    self.display.drawPixel(x,y)               
        print("Loaded map")
        return py_trees.common.Status.SUCCESS
    # Using shortest path move through map
    def update(self):
        self.display.setColor(0x0000FF)
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        px, py = world2map(xw, yw)
        self.start = (px, py)
        print("Start position: ", self.start)
        print("Goal position: ", self.goal)
        path = getShortestPath(self.map,self.start,self.goal)
        path = np.concatenate((path,np.flip(path,0)),axis=0)
        x = len(path)
        for i in range(x):
            if (((path[i,0],path[i,1]))==(self.goal[0],self.goal[1])):
                self.WP.append(self.end)
                self.display.drawPixel(self.end[0],self.end[1])
                break
            self.WP.append(map2world(path[i,0],path[i,1]))
            self.display.drawPixel(path[i,0],path[i,1])                   
        self.blackboard.write('waypoints',self.WP)
        return py_trees.common.Status.SUCCESS  
    
    def terminate(self, new_status):
        return py_trees.common.Status.SUCCESS    
        

