import numpy as np
from typing import List
from pythonFolder.pythonsrc.rod_mechanics.base_force import BaseForce
from soft_robots import SoftRobots

class ElasticTwistingForce(BaseForce):
    def __init__(self, m_soft_robots: List[SoftRobots]):
        super().__init__(m_soft_robots)

        self.grad_twists: List[List[np.ndarray]] = []
        self.deltams: List[List[np.ndarray]] = []
        self.theta_fs: List[List[np.ndarray]] = []
        self.theta_es: List[List[np.ndarray]] = []

        for limb in super().soft_robots.limbs:
            self.grad_twists.append(np.zeros((limb.nv, 11)))  # GradTwists for the vertices of the limb
            self.deltams.append(np.zeros(limb.ne))  # Delta m values for edges
            self.theta_fs.append(np.zeros(limb.ne))  # Final twist angles for edges
            self.theta_es.append(np.zeros(limb.ne))  # Elastic twist angles for edges

        for joint in super().soft_robots.joints:
            nb = joint.num_bending_combos  # Number of bending combos
            self.grad_twists.append(np.zeros((nb, 11)))  # GradTwists for the joint
            self.deltams.append(np.zeros(nb))  # Delta m values for the joint
            self.theta_fs.append(np.zeros(nb))  # Final twist angles for the joint
            self.theta_es.append(np.zeros(nb))  # Elastic twist angles for the joint

        self.dd_twist = np.zeros((11, 11))
        self.JTT = np.zeros((11, 11))
        self.grad_twist_local = np.zeros(11)
        self.f = np.zeros(11)

        self.ci : int= 0
        self.ind = self.ind1 = self.ind2 = 0
        self.norm_e = self.norm_f = 0.0
        self.norm2_e = self.norm2_f = 0.0
        self.value = self.chi = self.milen = 0.0
        self.t0 = self.t1 = np.zeros(3)
        self.te = self.tf = np.zeros(3)
        self.kb_local = np.zeros(3)
        self.tilde_t = np.zeros(3)
        self.theta_f: List[np.ndarray] = None
        self.theta_e: List[np.ndarray] = None
        self.deltam: List[np.ndarray] = None
        self.get_undeformed_twist = np.zeros(0)
        self.d2m_de2 = self.d2m_df2 = self.d2m_de_df = self.d2m_df_de = np.zeros((3, 3))
        self.te_matrix = np.zeros((3, 3))
        self.j = np.zeros((11, 11))
        self.grad_twist: List[np.ndarray] = None
        self.GJ = 0.0



    def compute_force(self, dt: float):
        for limb_idx, limb in enumerate(self.soft_robots.limbs):
            self.GJ = limb.GJ
            self.grad_twist = self.grad_twists[limb_idx]
            self.deltam = self.deltams[limb_idx]
            self.theta_f = self.theta_fs[limb_idx]
            self.theta_e = self.theta_es[limb_idx]

            for i in range(limb.ne):
                self.theta_f[i] = limb.x[4*i + 3]

            for i in range(limb.ne):
                self.theta_e[i] = 0 if i == 0 else self.theta_f[i-1]

            self.deltam[:] = self.theta_f - self.theta_e

            for i in range(1, limb.ne):
                self.norm_e = limb.edge_len[i-1]
                self.norm_f = limb.edge_len[i]
                self.grad_twist[i, :3] = -0.5 / self.norm_e * limb.kb[i]
                self.grad_twist[i, 8:11] = 0.5 / self.norm_f * limb.kb[i]
                self.grad_twist[i, 4:7] = -(self.grad_twist[i, :3] + self.grad_twist[i, 8:11])
                self.grad_twist[i, 3] = -1
                self.grad_twist[i, 7] = 1

            for i in range(1, limb.ne):
                self.value = self.GJ / limb.voronoi_len[i] * (self.deltam[i] + limb.ref_twist[i] - limb.twist_bar[i])
                self.f = -self.value * self.grad_twist[i]

                if limb.is_node_joint[i-1] != 1 and limb.is_node_joint[i] != 1 and limb.is_node_joint[i+1] != 1:
                    self.ci = 4*i - 4
                    for k in range(11):
                        self.ind = self.ci + k
                        super().stepper.add_force(self.ind, -self.f[k], limb_idx)
                else:
                    n1, l1 = limb.joint_ids[i-1]
                    n2, l2 = limb.joint_ids[i]
                    n3, l3 = limb.joint_ids[i+1]

                    for k in range(3):
                        super().stepper.add_force(4*n1+k, -self.f[k], l1)
                        super().stepper.add_force(4*n2+k, -self.f[k+4], l2)
                        super().stepper.add_force(4*n3+k, -self.f[k+8], l3)
                    
                    self.ci = 4*i - 4
                    super().stepper.add_force(self.ci+3, -self.f[3], limb_idx)
                    super().stepper.add_force(self.ci+7, -self.f[7], limb_idx)

        # Joint force computation
        for joint_idx, joint in enumerate(self.soft_robots.joints):
            self.GJ = self.soft_robots.limbs[0].GJ  # NOTE: Change this later
            self.grad_twist = self.grad_twists[len(self.soft_robots.limbs) + joint_idx]
            self.deltam = self.deltams[len(self.soft_robots.limbs) + joint_idx]
            self.theta_f = self.theta_fs[len(self.soft_robots.limbs) + joint_idx]
            self.theta_e = self.theta_es[len(self.soft_robots.limbs) + joint_idx]

            curr_iter = 0
            for i in range(joint.ne):
                l1 = joint.connected_nodes[i][1]
                for j in range(i+1, joint.ne):
                    l3 = joint.connected_nodes[j][1]

                    sgn1, sgn2 = joint.sgns[curr_iter]
                    theta1_i, theta2_i = joint.theta_inds[curr_iter]

                    self.theta_e[curr_iter] = self.soft_robots.limbs[l1].x[theta1_i] * sgn1
                    self.theta_f[curr_iter] = self.soft_robots.limbs[l3].x[theta2_i] * sgn2

                    curr_iter += 1

            self.deltam[:] = self.theta_f - self.theta_e

            curr_iter = 0
            for i in range(joint.ne):
                for j in range(i+1, joint.ne):
                    self.norm_e = joint.edge_len[i]
                    self.norm_f = joint.edge_len[j]
                    self.grad_twist[curr_iter, :3] = -0.5 / self.norm_e * joint.kb[curr_iter]
                    self.grad_twist[curr_iter, 8:11] = 0.5 / self.norm_f * joint.kb[curr_iter]
                    self.grad_twist[curr_iter, 4:7] = -(self.grad_twist[curr_iter, :3] + self.grad_twist[curr_iter, 8:11])
                    self.grad_twist[curr_iter, 3] = -1
                    self.grad_twist[curr_iter, 7] = 1
                    curr_iter += 1

            curr_iter = 0
            n2, l2 = joint.joint_node, joint.joint_limb
            for i in range(joint.ne):
                n1, l1 = joint.connected_nodes[i]
                for j in range(i+1, joint.ne):
                    n3, l3 = joint.connected_nodes[j]

                    sgn1, sgn2 = joint.sgns[curr_iter]
                    theta1_i, theta2_i = joint.theta_inds[curr_iter]

                    self.value = self.GJ / joint.voronoi_len[curr_iter] * (self.deltam[curr_iter] + 
                            joint.ref_twist[curr_iter] - joint.twist_bar[curr_iter])
                    f = -self.value * self.grad_twist[curr_iter]

                    for k in range(3):
                        super().stepper.add_force(4*n1+k, -self.f[k], l1)
                        super().stepper.add_force(4*n2+k, -self.f[k+4], l2)
                        super().stepper.add_force(4*n3+k, -self.f[k+8], l3)
                    
                    super().stepper.add_force(theta1_i, -self.f[3] * sgn1, l1)
                    super().stepper.add_force(theta2_i, -self.f[7] * sgn2, l3)

                    curr_iter += 1

    def compute_force_and_jacobian(self, dt: float):
        self.compute_force(dt)

        limb_idx = 0
        for limb in super().soft_robots.limbs:
            self.GJ = limb.GJ
            self.grad_twist = self.grad_twists[limb_idx]
            self.deltam = self.deltams[limb_idx]

            for i in range(1, limb.ne):
                self.norm_e = limb.edge_len[i - 1]
                self.norm_f = limb.edge_len[i]
                self.te = limb.tangent[i - 1, :]
                self.tf = limb.tangent[i, :]

                self.norm2_e = self.norm_e * self.norm_e
                self.norm2_f = self.norm_f * self.norm_f

                self.kbLocal = limb.kb[i, :]

                self.chi = 1.0 + np.dot(self.te, self.tf)
                self.tilde_t = (self.te + self.tf) / self.chi

                # Create cross product matrix for te
                self.teMatrix = np.cross(np.identity(3), self.te)

                self.d2m_de2 = -0.25 / self.norm2_e * (np.dot(self.kbLocal, (self.te + self.tilde_t).T) + np.dot((self.te + self.tilde_t), self.kbLocal.T))
                self.d2m_df2 = -0.25 / self.norm2_f * (np.dot(self.kbLocal, (self.tf + self.tilde_t).T) + np.dot((self.tf + self.tilde_t), self.kbLocal.T))
                self.d2m_de_df = 0.5 / (self.norm_e * self.norm_f) * (2.0 / self.chi * self.teMatrix - np.dot(self.kbLocal, self.tilde_t.T))
                self.d2m_df_de = self.d2m_de_df.T

                # Assigning values to DDtwist block matrix
                self.dd_twist[0:3, 0:3] = self.d2m_de2
                self.dd_twist[0:3, 4:7] = -self.d2m_de2 + self.d2m_de_df
                self.dd_twist[4:7, 0:3] = -self.d2m_de2 + self.d2m_df_de
                self.dd_twist[4:7, 4:7] = self.d2m_de2 - (self.d2m_de_df + self.d2m_df_de) + self.d2m_df2
                self.dd_twist[0:3, 8:11] = -self.d2m_de_df
                self.dd_twist[8:11, 0:3] = -self.d2m_df_de
                self.dd_twist[8:11, 4:7] = self.d2m_df_de - self.d2m_df2
                self.dd_twist[4:7, 8:11] = self.d2m_de_df - self.d2m_df2
                self.dd_twist[8:11, 8:11] = self.d2m_df2

                self.grad_twist_local = self.grad_twist[i, :]
                self.milen = -1 / limb.voronoi_len[i]

                # Jacobian matrix self.JTT calculation
                self.JTT = self.GJ * self.milen * ((self.deltam[i] + limb.ref_twist[i] - limb.twist_bar[i]) * self.dd_twist +
                                    np.outer(self.grad_twist_local, self.grad_twist_local))

                if limb.isNodeJoint[i - 1] != 1 and limb.isNodeJoint[i] != 1 and limb.isNodeJoint[i + 1] != 1:
                    for j in range(11):
                        for k in range(11):
                            ind1 = 4 * i - 4 + j
                            ind2 = 4 * i - 4 + k
                            super().stepper.add_jacobian(ind1, ind2, -self.JTT[k, j], limb_idx)
                else:
                    n1, l1 = limb.joint_ids[i - 1]
                    n2, l2 = limb.joint_ids[i]
                    n3, l3 = limb.joint_ids[i + 1]

                    for t in range(3):
                        for k in range(3):
                            super().stepper.add_jacobian(4 * n1 + t, 4 * n1 + k, -self.JTT[k, t], l1)
                            super().stepper.add_jacobian(4 * n1 + t, 4 * n2 + k, -self.JTT[k + 4, t], l1, l2)
                            super().stepper.add_jacobian(4 * n1 + t, 4 * n3 + k, -self.JTT[k + 8, t], l1, l3)

                            super().stepper.add_jacobian(4 * n2 + t, 4 * n1 + k, -self.JTT[k, t + 4], l2, l1)
                            super().stepper.add_jacobian(4 * n2 + t, 4 * n2 + k, -self.JTT[k + 4, t + 4], l2)
                            super().stepper.add_jacobian(4 * n2 + t, 4 * n3 + k, -self.JTT[k + 8, t + 4], l2, l3)

                            super().stepper.add_jacobian(4 * n3 + t, 4 * n1 + k, -self.JTT[k, t + 8], l3, l1)
                            super().stepper.add_jacobian(4 * n3 + t, 4 * n2 + k, -self.JTT[k + 4, t + 8], l3, l2)
                            super().stepper.add_jacobian(4 * n3 + t, 4 * n3 + k, -self.JTT[k + 8, t + 8], l3)

                    ci = 4 * (i - 1)
                    n1_i = 4 * n1
                    n2_i = 4 * n2
                    n3_i = 4 * n3
                    for k in range(3):
                        super().stepper.add_jacobian(ci + 3, n1_i + k, -self.JTT[k, 3], limb_idx, l1)
                        super().stepper.add_jacobian(n1_i + k, ci + 3, -self.JTT[3, k], l1, limb_idx)
                        super().stepper.add_jacobian(ci + 3, n2_i + k, -self.JTT[k + 4, 3], limb_idx, l2)
                        super().stepper.add_jacobian(n2_i + k, ci + 3, -self.JTT[3, k + 4], l2, limb_idx)
                        super().stepper.add_jacobian(ci + 3, n3_i + k, -self.JTT[k + 8, 3], limb_idx, l3)
                        super().stepper.add_jacobian(n3_i + k, ci + 3, -self.JTT[3, k + 8], l3, limb_idx)

                        super().stepper.add_jacobian(ci + 7, n1_i + k, -self.JTT[k, 7], limb_idx, l1)
                        super().stepper.add_jacobian(n1_i + k, ci + 7, -self.JTT[7, k], l1, limb_idx)
                        super().stepper.add_jacobian(ci + 7, n2_i + k, -self.JTT[k + 4, 7], limb_idx, l2)
                        super().stepper.add_jacobian(n2_i + k, ci + 7, -self.JTT[7, k + 4], l2, limb_idx)
                        super().stepper.add_jacobian(ci + 7, n3_i + k, -self.JTT[k + 8, 7], limb_idx, l3)
                        super().stepper.add_jacobian(n3_i + k, ci + 7, -self.JTT[7, k + 8], l3, limb_idx)

                    super().stepper.add_jacobian(ci + 3, ci + 3, -self.JTT[3, 3], limb_idx)
                    super().stepper.add_jacobian(ci + 3, ci + 7, -self.JTT[7, 3], limb_idx)
                    super().stepper.add_jacobian(ci + 7, ci + 3, -self.JTT[3, 7], limb_idx)
                    super().stepper.add_jacobian(ci + 7, ci + 7, -self.JTT[7, 7], limb_idx)

            limb_idx += 1

        joint_idx = 0
        curr_iter = 0
        num_limbs = len(super().soft_robots.limbs)

        for joint in super().soft_robots.joints:
            curr_iter = 0
            self.GJ = super().soft_robots.limbs[0].GJ  # NOTE: CHANGE THIS LATER
            self.gradTwist = self.grad_twist[num_limbs + joint_idx]
            self.deltam = self.deltams[num_limbs + joint_idx]
            n2 = joint.joint_node
            l2 = joint.joint_limb

            for i in range(joint.ne):
                n1, l1 = joint.connected_nodes[i]
                for j in range(i + 1, joint.ne):
                    n3, l3 = joint.connected_nodes[j]

                    sgn1 = joint.sgns[curr_iter][0]
                    sgn2 = joint.sgns[curr_iter][1]
                    theta1_i = joint.theta_inds[curr_iter][0]
                    theta2_i = joint.theta_inds[curr_iter][1]

                    self.norm_e = joint.edge_len[i]
                    self.norm_f = joint.edge_len[j]
                    self.te = joint.tangents[i, :] * sgn1
                    self.tf = joint.tangents[j, :] * sgn2

                    self.norm2_e = self.norm_e * self.norm_e
                    self.norm2_f = self.norm_f * self.norm_f

                    self.kb_local = joint.kb[curr_iter, :]

                    self.chi = 1.0 + np.dot(self.te, self.tf)
                    self.tilde_t = (self.te + self.tf) / self.chi

                    teMatrix = np.cross(np.identity(3), self.te)

                    self.d2m_de2 = -0.25 / self.norm2_e * (np.dot(self.kb_local, (self.te + self.tilde_t).T) + np.dot((self.te + self.tilde_t), self.kb_local.T))
                    self.d2m_df2 = -0.25 / self.norm2_f * (np.dot(self.kb_local, (self.tf + self.tilde_t).T) + np.dot((self.tf + self.tilde_t), self.kb_local.T))
                    self.d2m_de_df = 0.5 / (self.norm_e * self.norm_f) * (2.0 / self.chi * teMatrix - np.dot(self.kbLocal, self.tilde_t.T))
                    self.d2m_df_de = self.d2m_de_df.T

                    self.dd_twist[0:3, 0:3] = self.d2m_de2
                    self.dd_twist[0:3, 4:7] = -self.d2m_de2 + self.d2m_de_df
                    self.dd_twist[4:7, 0:3] = -self.d2m_de2 + self.d2m_df_de
                    self.dd_twist[4:7, 4:7] = self.d2m_de2 - (self.d2m_de_df + self.d2m_df_de) + self.d2m_df2
                    self.dd_twist[0:3, 8:11] = -self.d2m_de_df
                    self.dd_twist[8:11, 0:3] = -self.d2m_df_de
                    self.dd_twist[8:11, 4:7] = self.d2m_df_de - self.d2m_df2
                    self.dd_twist[4:7, 8:11] = self.d2m_de_df - self.d2m_df2
                    self.dd_twist[8:11, 8:11] = self.d2m_df2

                    self.grad_twist_local = self.grad_twist[curr_iter, :]
                    self.milen = -1 / joint.voronoi_len[curr_iter]

                    self.JTT = self.GJ * self.milen * ((self.deltam[curr_iter] + joint.ref_twist[curr_iter] - joint.twist_bar[curr_iter]) * self.dd_twist +
                                        np.outer(self.grad_twist_local, self.grad_twist_local))

                    if sgn1 == -1:
                        self.JTT[:, 3] *= -1
                        self.JTT[3, :] *= -1
                    if sgn2 == -1:
                        self.JTT[:, 7] *= -1
                        self.JTT[7, :] *= -1

                    for t in range(3):
                        for k in range(3):
                            super().stepper.add_jacobian(4 * n1 + t, 4 * n1 + k, -self.JTT[k, t], l1)
                            super().stepper.add_jacobian(4 * n1 + t, 4 * n2 + k, -self.JTT[k + 4, t], l1, l2)
                            super().stepper.add_jacobian(4 * n1 + t, 4 * n3 + k, -self.JTT[k + 8, t], l1, l3)

                            super().stepper.add_jacobian(4 * n2 + t, 4 * n1 + k, -self.JTT[k, t + 4], l2, l1)
                            super().stepper.add_jacobian(4 * n2 + t, 4 * n2 + k, -self.JTT[k + 4, t + 4], l2)
                            super().stepper.add_jacobian(4 * n2 + t, 4 * n3 + k, -self.JTT[k + 8, t + 4], l2, l3)

                            super().stepper.add_jacobian(4 * n3 + t, 4 * n1 + k, -self.JTT[k, t + 8], l3, l1)
                            super().stepper.add_jacobian(4 * n3 + t, 4 * n2 + k, -self.JTT[k + 4, t + 8], l3, l2)
                            super().stepper.add_jacobian(4 * n3 + t, 4 * n3 + k, -self.JTT[k + 8, t + 8], l3)

                    for k in range(3):
                        super().stepper.add_jacobian(theta1_i, 4 * n1 + k, -self.JTT[k, 3], l1)
                        super().stepper.add_jacobian(4 * n1 + k, theta1_i, -self.JTT[3, k], l1)

                        super().stepper.add_jacobian(theta1_i, 4 * n2 + k, -self.JTT[k + 4, 3], l1, l2)
                        super().stepper.add_jacobian(4 * n2 + k, theta1_i, -self.JTT[3, k + 4], l2, l1)

                        super().stepper.add_jacobian(theta1_i, 4 * n3 + k, -self.JTT[k + 8, 3], l1, l3)
                        super().stepper.add_jacobian(4 * n3 + k, theta1_i, -self.JTT[3, k + 8], l3, l1)

                        super().stepper.add_jacobian(theta2_i, 4 * n1 + k, -self.JTT[k, 7], l3, l1)
                        super().stepper.add_jacobian(4 * n1 + k, theta2_i, -self.JTT[7, k], l1, l3)

                        super().stepper.add_jacobian(theta2_i, 4 * n2 + k, -self.JTT[k + 4, 7], l3, l2)
                        super().stepper.add_jacobian(4 * n2 + k, theta2_i, -self.JTT[7, k + 4], l2, l3)

                        super().stepper.add_jacobian(theta2_i, 4 * n3 + k, -self.JTT[k + 8, 7], l3)
                        super().stepper.add_jacobian(4 * n3 + k, theta2_i, -self.JTT[7, k + 8], l3)

                    super().stepper.add_jacobian(theta1_i, theta1_i, -self.JTT[3, 3], l1)
                    super().stepper.add_jacobian(theta1_i, theta2_i, -self.JTT[7, 3], l1, l3)
                    super().stepper.add_jacobian(theta2_i, theta1_i, -self.JTT[3, 7], l3, l1)
                    super().stepper.add_jacobian(theta2_i, theta2_i, -self.JTT[7, 7], l3)

                    curr_iter += 1

            joint_idx += 1

    @staticmethod
    def cross_mat(a: np.ndarray, b: np.ndarray):
        return np.array([
            [0, -a[2], a[1]],
            [a[2], 0, -a[0]],
            [-a[1], a[0], 0]
        ])