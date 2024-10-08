from pythonsrc.rod_mechanics.base_force import BaseForce


class GravityForce(BaseForce):
    def __init__(self, soft_robots, g_vector):
        super().__init__(soft_robots)
        self.g_vector = g_vector

    
