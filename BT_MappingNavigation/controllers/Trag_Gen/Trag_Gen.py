from os.path import exists
import numpy as np
from controller import Robot, Supervisor
import py_trees
from py_trees.composites import Sequence, Parallel, Selector
from mapping import Mapping
from navigation import Navigation
from planning import Planning

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
WP = [(0.612, -0.50),(0.612, -1.5),(0.612, -2.5),(0.04, -3.0),(-1.0, -3.0), (-1.65, -2.5),(-1.65, -1.5), (-1.65, -0.5), (-1.0, 0.25), (0, 0)]



class DoesMapExist(py_trees.behaviour.Behaviour):
    def update(self):
        file_exists = exists('cspace.npy')
        if(file_exists):
            print("Map already exists")
            return py_trees.common.Status.SUCCESS
        else:
            print("Map does not exist")
            return py_trees.common.Status.FAILURE
        
class Blackboard:
    def __init__(self):
        self.data = {}
    def write(self, key, value):
        self.data[key] = value
    def read(self, key):
        return self.data.get(key)

blackboard = Blackboard()
blackboard.write('robot',robot)
blackboard.write('waypoints',WP)


tree = Sequence("Main",children=[
        Selector("Does map exist?",children=[
            DoesMapExist("Test for map"),
            Parallel("Mapping",policy=py_trees.common.ParallelPolicy.SuccessOnOne(), children=[
                Mapping("map the environment",blackboard),
                Navigation("move around the table",blackboard)
                ])
            ],memory=True),
        Planning("compute path to lower left corner",blackboard,(-1.46,-3.12)),
        Navigation("move to lower left corner", blackboard),
        Planning("compute path to sink",blackboard,(0.88,0.09)),
        Navigation("move to sink",blackboard)
        ],memory=True)
            
tree.setup_with_descendants()

while robot.step(timestep) != -1:
    tree.tick_once()

