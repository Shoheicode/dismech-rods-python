from typing import List

from pythonsrc.rod_mechanics.elastic_rod import ElasticRod


class BaseController:
    def __init__(self, limbs: List[ElasticRod]):
        """
        Initialize the base controller with a list of elastic rods (limbs).
        
        Parameters:
            limbs (list of elasticRod): A list of elasticRod objects representing the limbs.
        """
        self.limbs = limbs
        self.num_actuators = len(limbs)
        self.current_time = 0.0

    def update_time_step(self, dt):
        """
        Update the current time by the time step `dt`.
        
        Parameters:
            dt (float): Time step for updating the controller.
        """
        self.current_time += dt
