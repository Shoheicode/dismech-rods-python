from typing import List
from pythonsrc.rod_mechanics.base_force import BaseForce
from pythonsrc.rod_mechanics.elastic_rod import ElasticRod
from pythonsrc.rod_mechanics.soft_robots import SoftRobots
import numpy as np

class elasticStretchingForce(BaseForce):
    
    def __init__(self, soft_robots: SoftRobots):
        self.len : float = 0
        self.ref_length :float = 0
        self.epsX: float = 0
        self.u = np.zeros(3)
        self.dxx = np.zeros(3)
        self.f = np.zeros(3)
        self.Id3 = np.eye(3)
        self.M0 = np.zeros((3, 3))
        self.v = np.zeros((1, 3))
        self.JSS = np.zeros((7,7))

        super().__init__(soft_robots)

        for limb in soft_robots.limbs:
            EI = limb.EI
            print(EI)

    def compute_force(self, dt):
        limb_idx = 0
        for limb in super().soft_robots.limbs:
            a : ElasticRod = limb
            for i in range(a.ne):
                if a.is_edge_joint[i]:
                    continue
                self.epsX = a.edge_len[i] / a.ref_len[i] - 1.0
                f = limb.EA * limb.tangent[i, :] * self.epsX  # NumPy row access

                # Apply forces
                for k in range(3):
                    ind = 4 * i + k
                    super().stepper.addForce(ind, -f[k], limb_idx)  # subtracting elastic force

                    ind = 4 * (i + 1) + k
                    super().stepper.addForce(ind, f[k], limb_idx)  # adding elastic force

            limb_idx += 1

                
    
    def compute_force_and_jacobian(self, dt):
        return super().compute_force_and_jacobian(dt)
