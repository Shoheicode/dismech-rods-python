import globalDefinitions
from abc import ABC, abstractmethod

class BaseForce(object):
    def __init__(self, soft_robots):
        print('none')
        self.soft_robots = soft_robots
        self.stepper = None
    
    @abstractmethod
    def compute_force(self, dt):
        pass

    @abstractmethod
    def compute_force_and_jacobian(self, dt):
        pass

    def set_time_stepper(self, stepper):
        self.stepper = stepper