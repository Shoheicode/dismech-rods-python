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

    def compute_force(self, dt):
        limb_idx = 0
        for limb in super().soft_robots.limbs:
            for i in range(limb.ne):
                if limb.is_edge_joint[i]:
                    continue
                self.epsX = limb.edge_len[i] / limb.ref_len[i] - 1.0
                self.f = limb.EA * limb.tangent[i, :] * self.epsX  # NumPy row access

                # Apply forces
                for k in range(3):
                    ind = 4 * i + k
                    super().stepper.addForce(ind, -self.f[k], limb_idx)  # subtracting elastic force

                    ind = 4 * (i + 1) + k
                    super().stepper.addForce(ind, self.f[k], limb_idx)  # adding elastic force

            limb_idx += 1
        
        for joint in super().soft_robots.joints:
            for i in range(joint.ne):
                sgn = 1 if joint.bending_twist_signs[i] == 1 else -1
                n1 = joint.connected_nodes[i][0]
                limb_idx = joint.connected_nodes[i][1]
                curr_limb = super().soft_robots.limbs[limb_idx]

                # Compute epsX and force 'f' for joint
                epsX = joint.edge_len[i] / joint.ref_len[i] - 1.0
                f = curr_limb.EA * joint.tangents[i, :] * sgn * self.epsX  # NumPy row access

                # Apply forces for the joint
                for k in range(3):
                    ind = 4 * n1 + k
                    super().stepper.addForce(ind, -self.f[k], limb_idx)

                    ind = 4 * joint.joint_node + k
                    super().stepper.addForce(ind, self.f[k], joint.joint_limb)
    
    def compute_force_and_jacobian(self, dt):
        self.compute_force(dt)
        limb_idx = 0
        for limb in super().soft_robots.limbs:
            for i in range(limb.ne):
                if limb.is_edge_joint[i]:
                    continue

                # Calculate lengths
                self.len = limb.edge_len[i]
                self.ref_length = limb.ref_len[i]
                 # Calculate the difference vector (dxx)
                self.dxx = np.zeros(3)
                self.dxx[0] = limb.x[4*i+4] - limb.x[4*i+0]
                self.dxx[1] = limb.x[4*i+5] - limb.x[4*i+1]
                self.dxx[2] = limb.x[4*i+6] - limb.x[4*i+2]

                # Define u and v
                self.u = self.dxx
                self.v = self.u.reshape(-1, 1)  # Column vector

                # Compute M0 matrix
                self.M0 = limb.EA * ((1 / self.ref_length - 1 / self.len) * self.Id3 + (1 / self.len) * np.outer(u, u) / (u.dot(u)))

                # Update Jss blocks
                self.JSS[0:3, 0:3] = -M0
                self.JSS[4:7, 4:7] = -M0
                self.JSS[4:7, 0:3] = M0
                self.JSS[0:3, 4:7] = M0

                # Add to the Jacobian matrix
                for j in range(7):
                    for k in range(7):
                        ind1 = 4*i + j
                        ind2 = 4*i + k
                        super().stepper.addJacobian(ind1, ind2, -self.JSS[k, j], limb_idx)

            limb_idx += 1
        # Process joints
        for joint in super().soft_robots.joints:
            for i in range(joint.ne):
                n1 = joint.connected_nodes[i][0]
                limb_idx = joint.connected_nodes[i][1]
                curr_limb = super().soft_robots.limbs[limb_idx]

                # Calculate lengths
                self.len = joint.edge_len[i]
                refLength = joint.ref_len[i]

                # Calculate the difference vector (dxx)
                self.dxx = np.zeros(3)
                self.dxx[0] = joint.x[0] - curr_limb.x[4*n1]
                self.dxx[1] = joint.x[1] - curr_limb.x[4*n1+1]
                self.dxx[2] = joint.x[2] - curr_limb.x[4*n1+2]

                # Define u and v
                self.u = self.dxx
                self.v = self.u.reshape(-1, 1)  # Column vector

                # Compute M0 matrix
                self.M0 = curr_limb.EA * ((1 / refLength - 1 / self.len) * self.Id3 + (1 / self.len) * np.outer(self.u, self.u) / (self.u.dot(self.u)))

                # Update Jss blocks
                self.JSS[0:3, 0:3] = -self.M0
                self.JSS[4:7, 4:7] = -self.M0
                self.JSS[0:3, 4:7] = self.M0
                self.JSS[4:7, 0:3] = self.M0

                # Apply the Jacobian using n1 and joint node
                l1 = limb_idx
                l2 = joint.joint_limb
                n2 = joint.joint_node

                for j in range(3):
                    for k in range(3):
                        super().stepper.addJacobian(4*n1 + j, 4*n1 + k, -self.JSS[k, j], l1)
                        super().stepper.addJacobian(4*n1 + j, 4*n2 + k, -self.JSS[k + 4, j], l1, l2)
                        super().stepper.addJacobian(4*n2 + j, 4*n1 + k, -self.JSS[k, j + 4], l2, l1)
                        super().stepper.addJacobian(4*n2 + j, 4*n2 + k, -self.JSS[k + 4, j + 4], l2)