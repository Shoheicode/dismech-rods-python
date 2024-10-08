from pythonsrc.rod_mechanics.base_force import BaseForce
from pythonsrc.rod_mechanics.soft_robots import SoftRobots


class elasticBendingForce(BaseForce):
    def __init__(self, soft_robots: SoftRobots):
        super().__init__(soft_robots)
        for limb in soft_robots.limbs:
            EI = limb


    def compute_force(self, dt):
        return super().compute_force(dt)
    
    def compute_force_and_jacobian(self, dt):
        return super().compute_force_and_jacobian(dt)
