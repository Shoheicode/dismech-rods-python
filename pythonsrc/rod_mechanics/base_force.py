import globalDefinitions
from abc import ABC, abstractmethod

class BaseForce:
    def __init__(self, soft_robots):
        print('none')
        self.soft_robots = soft_robots
        self.stepper = None
    

    def computeForce(self, dt):
        pass