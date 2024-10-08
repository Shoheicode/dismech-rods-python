from pythonsrc.rod_mechanics.base_force import BaseForce


class elasticBendingForce(BaseForce):
    def __init__(self, soft_robots):
        super().__init__(soft_robots)

    def compute_force(self, dt):
        return super().compute_force(dt)
    
    def compute_force_and_jacobian(self, dt):
        return super().compute_force_and_jacobian(dt)
