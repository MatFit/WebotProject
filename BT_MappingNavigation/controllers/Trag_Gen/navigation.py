import py_trees
from py_trees import common
import numpy as np


# def world2map(xw,yw):
#    px = int ((xw+2.25)*40)
#    py = int ((yw-2)*(-50))

#    px = min(px,199)
#    py = min(px,299)
#    px = max(px,0)
#    py = max(py,0)

#    return [px,py]


# def map2world(px,py):
#    xw = px/40-2.25
#    yw=py/(-50)+2
#    return [xw,yw]


class Navigation(py_trees.behaviour.Behaviour):
    #Initialize
    def __init__(self, name, blackboard):
        super(Navigation, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        
    # Setting up movements  
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)       

        self.leftMotor = self.robot.getDevice('wheel_left_joint')
        self.rightMotor = self.robot.getDevice('wheel_right_joint')
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))

        self.marker = self.robot.getFromDef('marker').getField('translation')
    # Set up motors
    def initialise(self):
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)

        self.index = 0        

        self.WP = self.blackboard.read('waypoints')

    # Navigate through enviorment
    def update(self):

        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        theta = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])  

        rho = np.sqrt((xw-self.WP[self.index][0])**2+(yw-self.WP[self.index][1])**2)
        alpha = np.arctan2(self.WP[self.index][1]-yw, self.WP[self.index][0]-xw)-theta 

        if (alpha > np.pi):
            alpha = alpha - 2 * np.pi
        elif (alpha < -np.pi):
            alpha = alpha + 2 * np.pi


        self.marker.setSFVec3f([*self.WP[self.index],0])  


        vL = 6.28
        vR = 6.28

        p1 = 4
        p2 = 2

        vL = -p1 * alpha + p2 * rho
        vR = p1 * alpha + p2 * rho   

        vL = min(vL, 6.28)
        vR = min(vR, 6.28)
        vL = max(vL, -6.28)
        vR = max(vR, -6.28)

        self.leftMotor.setVelocity(vL)
        self.rightMotor.setVelocity(vR)

        if (rho < 0.4):
            print("Reached Waypoint", self.index, len(self.WP))
            self.index = self.index + 1
            if self.index == len(self.WP):
                self.feedback_message = 'Last waypoint reached'
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.RUNNING


    def terminate(self, new_status):
        #stop the robot when reaching the last waypoint
        if self.index == len(self.WP):
            self.leftMotor.setVelocity(0)
            self.rightMotor.setVelocity(0)           
            return py_trees.common.Status.SUCCESS
           
            




        

   