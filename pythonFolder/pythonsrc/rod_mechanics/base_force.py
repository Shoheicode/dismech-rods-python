from abc import ABC, abstractmethod

from pythonsrc.rod_mechanics.soft_robots import SoftRobots
from pythonsrc.time_stepper.basetimestepper import BaseTimeStepper

class BaseForce(object):
    def __init__(self, soft_robots: SoftRobots):
        print('none')
        self.soft_robots = soft_robots
        self.stepper:BaseTimeStepper = None
    
    @abstractmethod
    def compute_force(self, dt):
        pass

    @abstractmethod
    def compute_force_and_jacobian(self, dt):
        pass

    def set_time_stepper(self, stepper: BaseTimeStepper):
        self.stepper = stepper