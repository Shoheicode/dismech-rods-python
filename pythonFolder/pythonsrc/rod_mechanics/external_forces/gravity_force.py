from pythonsrc.rod_mechanics.base_force import BaseForce
import numpy as np

class GravityForce(BaseForce):
    def __init__(self, soft_robots, g_vector):
        super().__init__(soft_robots)
        self.soft_robots = soft_robots
        self.g_vector = np.array(g_vector)
        self.mass_gravities = []
        self.set_gravity()

    def compute_force(self, dt):
        # print("HEYO")
        for limb_idx, limb in enumerate(self.soft_robots.limbs):
            mass_gravity = self.mass_gravities[limb_idx]
            for i in range(limb.ndof):
                if limb.is_dof_joint[i]:
                    continue
                self.stepper.add_force(i, -mass_gravity[i], limb_idx)  # Subtract gravity force
                # print("MASS GRAVITY", -mass_gravity[i])

        # For joints
        for joint in self.soft_robots.joints:
            for i in range(3):
                force = self.g_vector[i] * joint.mass
                self.stepper.add_force(4 * joint.joint_node + i, -force, joint.joint_limb)

    def compute_force_and_jacobian(self, dt):
        """
        Compute the force and its Jacobian (not implemented separately)
        :param dt: time step
        """
        self.compute_force(dt)

    def set_gravity(self):
        # print("SET GRAVITY FORCE")
        for limb in self.soft_robots.limbs:
            mass_gravity = np.zeros(limb.ndof)
            for i in range(limb.nv):
                for k in range(3):
                    ind = 4 * i + k
                    mass_gravity[ind] = self.g_vector[k] * limb.mass_array[ind]
            self.mass_gravities.append(mass_gravity)