import py_trees
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy import signal

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




class Mapping(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Mapping, self).__init__(name)
        self.hasrun = False
        self.robot = blackboard.read('robot')
    # Initialize robot
    def setup(self):
        self.logger.debug("map setup")
        self.timestep = int(self.robot.getBasicTimeStep())
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        self.display = self.robot.getDevice("display")

        self.lidar = self.robot.getDevice("Hokuyo URG-04LX-UG01")
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()
        
    # Initialize map
    def initialise(self):
        self.logger.debug("map initialized")
        self.map = np.zeros((200,300))
        self.angles = np.linspace(4.19/2,-4.19/2,667) 
        self.angles = self.angles[80:len(self.angles)-80]   
    # Create Map 
    def update(self):
        self.hasrun = True
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        theta=np.arctan2(self.compass.getValues()[0],self.compass.getValues()[1])
        
        px, py = world2map(xw,yw)
        self.display.setColor(0xFF0000)
        self.display.drawPixel(px,py)
        
        


        
        w_T_r = np.array([[np.cos(theta),-np.sin(theta),xw],
                      [np.sin(theta),np.cos(theta),yw],
                      [0,0,1]])
        
        ranges = np.array(self.lidar.getRangeImage())
        ranges = ranges[80:len(ranges)-80]
        ranges[ranges == np.inf] = 100
                      
        X_i = np.array([ranges*np.cos(self.angles)+0.202, ranges*np.sin(self.angles), np.ones((507,))]) 
        D = w_T_r @ X_i
        
        #Convert  world coordinates to map coordinates to display
        for d in D.transpose():
            Dx, Dy = world2map(d[0],d[1])
            self.map[px,py]+=0.01
            if(self.map[px,py]>1):
                self.map[px,py]=1
            v=int(self.map[px,py]*255)
            color=(v*256**2+v*256+v)
            self.display.setColor(int(color))
            self.display.drawPixel(px,py)
            
        return py_trees.common.Status.RUNNING
    
    # when map is finished
    def terminate(self, new_status):
        if(self.hasrun):
            cmap = signal.convolve2d(self.map,self.kernel,mode='same')
            cspace = cmap>0.9
            np.save('cspace',cspace)
   