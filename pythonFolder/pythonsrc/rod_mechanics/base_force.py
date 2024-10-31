from abc import ABC, abstractmethod

from pythonsrc.rod_mechanics.soft_robots import SoftRobots
# from pythonsrc.time_stepper.base_time_stepper import BaseTimeStepper

class BaseForce(object):
    def __init__(self, soft_robots: SoftRobots):
        print('none')
        self.soft_robots = soft_robots
        self.stepper= None
    
    @abstractmethod
    def compute_force(self, dt):
        pass

    @abstractmethod
    def compute_force_and_jacobian(self, dt):
        pass

    def getSoftRobots(self):
        return self.soft_robots
    
    def getStepper(self):
        return self.stepper

    def set_time_stepper(self, stepper):
        self.stepper = stepper