from pythonsrc.rod_mechanics.base_force import BaseForce


class GravityForce(BaseForce):
    def __init__(self, soft_robots, g_vector):
        super().__init__(soft_robots)
        self.g_vector = g_vector
        self.setGravity()

    def computeForce(self, dt):
        limb_idx = 0
        for limb in super().soft_robots:
            mass_gravity = mass_gravity[limb_idx]

    def setGravity(self):
        for limb in super().soft_robots:
            self.mass_gravity = 0