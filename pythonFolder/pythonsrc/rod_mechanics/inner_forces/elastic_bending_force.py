from typing import List
from pythonsrc.rod_mechanics.base_force import BaseForce
from pythonsrc.rod_mechanics.elastic_rod import ElasticRod
from pythonsrc.rod_mechanics.soft_robots import SoftRobots
import numpy as np

class ElasticBendingForce(BaseForce):
    ci = 0
    chi = 0.0
    len = 0.0
    kappa1 = 0.0
    kappa2 = 0.0
    norm_e = 0.0
    norm_f = 0.0
    norm2_e = 0.0
    norm2_f = 0.0

    # Initialize vectors as numpy arrays
    t0 = np.zeros(3)
    t1 = np.zeros(3)
    te = np.zeros(3)
    tf = np.zeros(3)
    d1e = np.zeros(3)
    d1f = np.zeros(3)
    d2e = np.zeros(3)
    d2f = np.zeros(3)
    tilde_t = np.zeros(3)
    tilde_d1 = np.zeros(3)
    tilde_d2 = np.zeros(3)
    Dkappa1De = np.zeros(3)
    Dkappa1Df = np.zeros(3)
    Dkappa2De = np.zeros(3)
    Dkappa2Df = np.zeros(3)
    kbLocal = np.zeros(3)
    kappaL = np.zeros(2)
    tmp = np.zeros(3)

    D2kappa1DeDthetae = np.zeros(3)
    D2kappa1DeDthetaf = np.zeros(3)
    D2kappa1DfDthetae = np.zeros(3)
    D2kappa1DfDthetaf = np.zeros(3)

    D2kappa2DeDthetae = np.zeros(3)
    D2kappa2DeDthetaf = np.zeros(3)
    D2kappa2DfDthetae = np.zeros(3)
    D2kappa2DfDthetaf = np.zeros(3)

    m1e = np.zeros(3)
    m2e = np.zeros(3)
    m1f = np.zeros(3)
    m2f = np.zeros(3)

    # Initialize matrices
    kappa1 = np.zeros((3, 3))
    kappa2 = np.zeros((3, 3))
    f = np.zeros(3)

    # Lists of matrices (using numpy arrays instead of Eigen::MatrixXd)
    gradKappa1s: List[np.ndarray] = []
    gradKappa2s: List[np.ndarray] = []

    gradKappa1 = np.zeros((3, 3))
    gradKappa2 = np.zeros((3, 3))
    relevantPart = np.zeros((3, 3))
    kappa = np.zeros((3, 3))
    DDkappa1 = np.zeros((11, 11))
    DDkappa2 = np.zeros((11, 11))
    Jbb = np.zeros((3, 3))

    EIMatrices: List[np.ndarray] = []

    """
        Creates a 3 x 3 matrix
        | 1, 0, 0 |
        | 0, 1, 0 |
        | 0, 0, 1 |

    """
    Id3 = np.eye(3)
    
    tt_o_tt = np.zeros((3, 3))
    tilde_d1_3d = np.zeros((3, 3))
    tilde_d2_3d = np.zeros((3, 3))
    tf_c_d2t_o_tt = np.zeros((3, 3))
    tt_o_tf_c_d2t = np.zeros((3, 3))
    kb_o_d2e = np.zeros((3, 3))
    d2e_o_kb = np.zeros((3, 3))
    D2kappa1De2 = np.zeros((3, 3))
    te_c_d2t_o_tt = np.zeros((3, 3))
    tt_o_te_c_d2t = np.zeros((3, 3))
    kb_o_d2f = np.zeros((3, 3))
    d2f_o_kb = np.zeros((3, 3))
    D2kappa1Df2 = np.zeros((3, 3))
    D2kappa1DeDf = np.zeros((3, 3))
    D2kappa1DfDe = np.zeros((3, 3))
    tf_c_d1t_o_tt = np.zeros((3, 3))
    tt_o_tf_c_d1t = np.zeros((3, 3))
    kb_o_d1e = np.zeros((3, 3))
    d1e_o_kb = np.zeros((3, 3))
    D2kappa2De2 = np.zeros((3, 3))
    te_c_d1t_o_tt = np.zeros((3, 3))
    tt_o_te_c_d1t = np.zeros((3, 3))
    kb_o_d1f = np.zeros((3, 3))
    d1f_o_kb = np.zeros((3, 3))
    D2kappa2Df2 = np.zeros((3, 3))
    D2kappa2DeDf = np.zeros((3, 3))
    D2kappa2DfDe= np.zeros((3, 3))
    
    def __init__(self, soft_robots: SoftRobots):
        super().__init__(soft_robots)
        
        for limb in self.getSoftRobots().limbs:
            EI = limb.EI
            # Create a 2x2 matrix EIMat and append to EIMatrices
            EIMat = np.array([[EI, 0],
                            [0, EI]])
            self.EIMatrices.append(EIMat)

            # Assuming limb is an object with an attribute 'nv'
            nv = limb.nv  # Number of vertices (nv) from the limb object

            # Append a zero matrix of shape (nv, 11) to both gradKappa1s and gradKappa2s
            self.gradKappa1s.append(np.zeros((nv, 11)))
            self.gradKappa2s.append(np.zeros((nv, 11)))
            # print(limb)
            # print("HIHIHI")

        for joint in self.getSoftRobots().joints:
            nb = joint.num_bending_combos
            self.gradKappa1s.append(np.zeros((nb, 11)))
            self.gradKappa2s.append(np.zeros((nb, 11)))
        
        self.relevantPart = np.zeros((11, 2))  # Equivalent to MatrixXd::Zero(11, 2)
        self.DDkappa1 = np.zeros((11, 11)) # Equivalent to MatrixXd::Zero(11, 11)
        self.DDkappa2 = np.zeros((11, 11)) # Equivalent to MatrixXd::Zero(11, 11)
        self.Jbb = np.zeros((11,11)) # Equivalent to MatrixXd::Zero(11, 11)

        # Initialize 3x3 matrices for D2kappa terms using np.zeros
        self.D2kappa1De2 = np.zeros((3, 3))    # Equivalent to D2kappa1De2.setZero(3, 3)
        self.D2kappa1Df2 = np.zeros((3, 3))    # Equivalent to D2kappa1Df2.setZero(3, 3)
        self.D2kappa1DeDf = np.zeros((3, 3))   # Equivalent to D2kappa1DeDf.setZero(3, 3)
        self.D2kappa2De2 = np.zeros((3, 3))    # Equivalent to D2kappa2De2.setZero(3, 3)
        self.D2kappa2Df2 = np.zeros((3, 3))    # Equivalent to D2kappa2Df2.setZero(3, 3)
        self.D2kappa2DeDf = np.zeros((3, 3))   # Equivalent to D2kappa2DeDf.setZero(3, 3)

        # Initialize vector f using np.zeros, equivalent to VectorXd::Zero in Eigen
        self.f = np.zeros(11)  # Equivalent to VectorXd::Zero(11)

    def compute_force(self, dt):
        limb_idx = 0

        for limb in self.getSoftRobots().limbs:
            self.gradKappa1 = self.gradKappa1s[limb_idx]
            self.gradKappa2 = self.gradKappa2s[limb_idx]

            for i in range(1, limb.ne):
                self.norm_e = limb.edge_len[i - 1]
                self.norm_f = limb.edge_len[i]
                self.te = limb.tangent[i - 1, :]
                self.tf = limb.tangent[i, :]
                self.d1e = limb.m1[i - 1, :]
                self.d2e = limb.m2[i - 1, :]
                self.d1f = limb.m1[i, :]
                self.d2f = limb.m2[i, :]

                self.chi = 1.0 + np.dot(self.te, self.tf)
                self.tilde_t = (self.te + self.tf) / self.chi
                self.tilde_d1 = (self.d1e + self.d1f) / self.chi
                self.tilde_d2 = (self.d2e + self.d2f) / self.chi

                self.kappa1 = limb.kappa[i, 0]
                self.kappa2 = limb.kappa[i, 1]

                self.Dkappa1De = (1.0 / self.norm_e) * (-self.kappa1 * self.tilde_t + np.cross(self.tf, self.tilde_d2))
                self.Dkappa1Df = (1.0 / self.norm_f) * (-self.kappa1 * self.tilde_t - np.cross(self.te, self.tilde_d2))
                self.Dkappa2De = (1.0 / self.norm_e) * (-self.kappa2 * self.tilde_t - np.cross(self.tf, self.tilde_d1))
                self.Dkappa2Df = (1.0 / self.norm_f) * (-self.kappa2 * self.tilde_t + np.cross(self.te, self.tilde_d1))

                self.gradKappa1[i, :3] = -self.Dkappa1De
                self.gradKappa1[i, 4:7] = self.Dkappa1De - self.Dkappa1Df
                self.gradKappa1[i, 8:11] = self.Dkappa1Df

                self.gradKappa2[i, :3] = -self.Dkappa2De
                self.gradKappa2[i, 4:7] = self.Dkappa2De - self.Dkappa2Df
                self.gradKappa2[i, 8:11] = self.Dkappa2Df

                self.kb_local = limb.kb[i, :]

                self.gradKappa1[i, 3] = -0.5 * np.dot(self.kb_local, self.d1e)
                self.gradKappa1[i, 7] = -0.5 * np.dot(self.kb_local, self.d1f)
                self.gradKappa2[i, 3] = -0.5 * np.dot(self.kb_local, self.d2e)
                self.gradKappa2[i, 7] = -0.5 * np.dot(self.kb_local, self.d2f)

            # Second loop
            for i in range(1, limb.ne):
                self.relevantPart[:, 0] = self.gradKappa1[i, :]
                self.relevantPart[:, 1] = self.gradKappa2[i, :]
                self.kappaL = limb.kappa[i, :] - limb.kappa_bar[i, :]

                self.f = -np.dot(self.relevantPart, np.dot(self.EIMatrices[limb_idx], self.kappaL)) / limb.voronoi_len[i]

                if limb.is_node_joint[i - 1] != 1 and limb.is_node_joint[i] != 1 and limb.is_node_joint[i + 1] != 1:
                    self.ci = 4 * i - 4
                    for k in range(11):
                        ind = self.ci + k
                        self.stepper.add_force(ind, -self.f[k], limb_idx)
                else:
                    n1, l1 = limb.joint_ids[i - 1]
                    n2, l2 = limb.joint_ids[i]
                    n3, l3 = limb.joint_ids[i + 1]

                    for k in range(3):
                        self.stepper.add_force(4 * n1 + k, -self.f[k], l1)
                        self.stepper.add_force(4 * n2 + k, -self.f[k + 4], l2)
                        self.stepper.add_force(4 * n3 + k, -self.f[k + 8], l3)

                    ci = 4 * i - 4
                    self.stepper.add_force(ci + 3, -self.f[3], limb_idx)
                    self.stepper.add_force(ci + 7, -self.f[7], limb_idx)

            limb_idx += 1

        joint_idx = 0
        num_limbs = len(self.soft_robots.limbs)

        for joint in self.soft_robots.joints:
            curr_iter = 0
            self.gradKappa1 = self.gradKappa1s[num_limbs + joint_idx]
            self.gradKappa2 = self.gradKappa2s[num_limbs + joint_idx]

            for i in range(joint.ne):
                sgn1 = 1 if joint.bending_twist_signs[i] == 1 else -1
                for j in range(i + 1, joint.ne):
                    sgn2 = -1 if joint.bending_twist_signs[j] == 1 else 1
                    self.norm_e = joint.edge_len[i]
                    self.norm_f = joint.edge_len[j]
                    self.te = sgn1 * joint.tangents[i, :]
                    self.tf = sgn2 * joint.tangents[j, :]
                    self.d1e = joint.m1[curr_iter, 0, :]
                    self.d2e = joint.m2[curr_iter, 0, :]
                    self.d1f = joint.m1[curr_iter, 1, :]
                    self.d2f = joint.m2[curr_iter, 1, :]

                    self.chi = 1.0 + np.dot(self.te, self.tf)
                    self.tilde_t = (self.te + self.tf) / self.chi
                    self.tilde_d1 = (self.d1e + self.d1f) / self.chi
                    self.tilde_d2 = (self.d2e + self.d2f) / self.chi

                    self.kappa1 = joint.kappa[curr_iter, 0]
                    self.kappa2 = joint.kappa[curr_iter, 1]

                    self.Dkappa1De = (1.0 / self.norm_e) * (-self.kappa1 * self.tilde_t + np.cross(self.tf, self.tilde_d2))
                    self.Dkappa1Df = (1.0 / self.norm_f) * (-self.kappa1 * self.tilde_t - np.cross(self.te, self.tilde_d2))
                    self.Dkappa2De = (1.0 / self.norm_e) * (-self.kappa2 * self.tilde_t - np.cross(self.tf, self.tilde_d1))
                    self.Dkappa2Df = (1.0 / self.norm_f) * (-self.kappa2 * self.tilde_t + np.cross(self.te, self.tilde_d1))

                    self.gradKappa1[curr_iter, :3] = -self.Dkappa1De
                    self.gradKappa1[curr_iter, 4:7] = self.Dkappa1De - self.Dkappa1Df
                    self.gradKappa1[curr_iter, 8:11] = self.Dkappa1Df

                    self.gradKappa2[curr_iter, :3] = -self.Dkappa2De
                    self.gradKappa2[curr_iter, 4:7] = self.Dkappa2De - self.Dkappa2Df
                    self.gradKappa2[curr_iter, 8:11] = self.Dkappa2Df

                    self.kbLocal = joint.kb[curr_iter, :]

                    self.gradKappa1[curr_iter, 3] = -0.5 * np.dot(self.kb_local, self.d1e)
                    self.gradKappa1[curr_iter, 7] = -0.5 * np.dot(self.kb_local, self.d1f)
                    self.gradKappa2[curr_iter, 3] = -0.5 * np.dot(self.kb_local, self.d2e)
                    self.gradKappa2[curr_iter, 7] = -0.5 * np.dot(self.kb_local, self.d2f)

                    curr_iter += 1

            curr_iter = 0
            n2 = joint.joint_node
            l2 = joint.joint_limb

            for i in range(joint.ne):
                n1 = joint.connected_nodes[i][0]
                l1 = joint.connected_nodes[i][1]

                for j in range(i + 1, joint.ne):
                    n3 = joint.connected_nodes[j][0]
                    l3 = joint.connected_nodes[j][1]

                    sgn1 = joint.sgns[curr_iter][0]
                    sgn2 = joint.sgns[curr_iter][1]
                    theta1_i = joint.theta_inds[curr_iter][0]
                    theta2_i = joint.theta_inds[curr_iter][1]

                    self.relevantPart[:, 0] = self.gradKappa1[curr_iter, :]
                    self.relevantPart[:, 1] = self.gradKappa2[curr_iter, :]
                    self.kappaL = joint.kappa[curr_iter, :] - joint.kappa_bar[curr_iter, :]

                    self.f = -np.dot(self.relevantPart, np.dot(self.EIMatrices[0], self.kappaL)) / joint.voronoi_len[curr_iter]

                    for k in range(3):
                        self.stepper.add_force(4 * n1 + k, -self.f[k], l1)
                        self.stepper.add_force(4 * n2 + k, -self.f[k + 4], l2)
                        self.stepper.add_force(4 * n3 + k, -self.f[k + 8], l3)

                    self.stepper.add_force(theta1_i, -self.f[3] * sgn1, l1)
                    self.stepper.add_force(theta2_i, -self.f[7] * sgn2, l3)

                    curr_iter += 1

            joint_idx += 1
    
    def compute_force_and_jacobian(self, dt):
        print("ELASTIC BENDING FORCE ")
        self.compute_force(dt)

        # Iterate through the limbs of the soft robot
        limb_idx = 0
        for limb in self.soft_robots.limbs:
            self.gradKappa1 = self.gradKappa1s[limb_idx]
            self.gradKappa2 = self.gradKappa2s[limb_idx]
            
            for i in range(1, limb.ne):
                self.norm_e = limb.edge_len[i - 1]
                self.norm_f = limb.edge_len[i]
                self.te = limb.tangent[i - 1]
                self.tf = limb.tangent[i]
                self.d1e = limb.m1[i - 1]
                self.d2e = limb.m2[i - 1]
                self.d1f = limb.m1[i]
                self.d2f = limb.m2[i]

                self.norm2_e = self.norm_e ** 2
                self.norm2_f = self.norm_f ** 2

                self.chi = 1.0 + np.dot(self.te, self.tf)
                self.tilde_t = (self.te + self.tf) / self.chi
                self.tilde_d1 = (self.d1e + self.d1f) / self.chi
                self.tilde_d2 = (self.d2e + self.d2f) / self.chi

                self.kappa1 = limb.kappa[i, 0]
                self.kappa2 = limb.kappa[i, 1]

                self.kbLocal = limb.kb[i]

                # Compute Jacobians
                self.jacobian_computation()

                self.len = limb.voronoi_len[i]
                self.relevantPart[:,0] = self.gradKappa1[i,:]
                self.relevantPart[:,1] = self.gradKappa2[i,:]

                # Compute kappaL
                self.kappaL = limb.kappa[i] - limb.kappa_bar[i]

                temp = -1.0 / limb.voronoi_len[i] * np.dot(self.kappaL.T, self.EIMatrices[limb_idx])
                
                self.Jbb += temp[0] * self.DDkappa1 + temp[1] * self.DDkappa2

                # Add Jacobians depending on node joints
                if limb.is_node_joint[i-1] != 1 and limb.is_node_joint[i] != 1 and limb.is_node_joint[i+1] != 1:
                    for j in range(11):
                        for k in range(11):
                            ind1 = 4 * i - 4 + j
                            ind2 = 4 * i - 4 + k
                            self.stepper.add_jacobian(ind1, ind2, -self.Jbb[k, j], limb_idx)
                else:
                    n1 = limb.joint_ids[i - 1][0]
                    l1 = limb.joint_ids[i - 1][1]
                    n2 = limb.joint_ids[i][0]
                    l2 = limb.joint_ids[i][1]
                    n3 = limb.joint_ids[i + 1][0]
                    l3 = limb.joint_ids[i + 1][1]

                    for t in range(3):
                        for k in range(3):
                            self.stepper.add_jacobian(4 * n1 + t, 4 * n1 + k, -self.Jbb[k, t], l1)
                            self.stepper.add_jacobian(4 * n1 + t, 4 * n2 + k, -self.Jbb[k + 4, t], l1, l2)
                            self.stepper.add_jacobian(4 * n1 + t, 4 * n3 + k, -self.Jbb[k + 8, t], l1, l3)

                            self.stepper.add_jacobian(4 * n2 + t, 4 * n1 + k, -self.Jbb[k, t + 4], l2, l1)
                            self.stepper.add_jacobian(4 * n2 + t, 4 * n2 + k, -self.Jbb[k + 4, t + 4], l2)
                            self.stepper.add_jacobian(4 * n2 + t, 4 * n3 + k, -self.Jbb[k + 8, t + 4], l2, l3)

                            self.stepper.add_jacobian(4 * n3 + t, 4 * n1 + k, -self.Jbb[k, t + 8], l3, l1)
                            self.stepper.add_jacobian(4 * n3 + t, 4 * n2 + k, -self.Jbb[k + 4, t + 8], l3, l2)
                            self.stepper.add_jacobian(4 * n3 + t, 4 * n3 + k, -self.Jbb[k + 8, t + 8], l3)

                    ci = 4 * (i - 1)
                    n1_i = 4 * n1
                    n2_i = 4 * n2
                    n3_i = 4 * n3

                    for k in range(3):
                        self.stepper.add_jacobian(ci + 3, n1_i + k, -self.Jbb[k, 3], limb_idx, l1)
                        self.stepper.add_jacobian(n1_i + k, ci + 3, -self.Jbb[3, k], l1, limb_idx)
                        self.stepper.add_jacobian(ci + 3, n2_i + k, -self.Jbb[k + 4, 3], limb_idx, l2)
                        self.stepper.add_jacobian(n2_i + k, ci + 3, -self.Jbb[3, k + 4], l2, limb_idx)
                        self.stepper.add_jacobian(ci + 3, n3_i + k, -self.Jbb[k + 8, 3], limb_idx, l3)
                        self.stepper.add_jacobian(n3_i + k, ci + 3, -self.Jbb[3, k + 8], l3, limb_idx)
                        self.stepper.add_jacobian(ci + 7, n1_i + k, -self.Jbb[k, 7], limb_idx, l1)
                        self.stepper.add_jacobian(n1_i + k, ci + 7, -self.Jbb[7, k], l1, limb_idx)
                        self.stepper.add_jacobian(ci + 7, n2_i + k, -self.Jbb[k + 4, 7], limb_idx, l2)
                        self.stepper.add_jacobian(n2_i + k, ci + 7, -self.Jbb[7, k + 4], l2, limb_idx)
                        self.stepper.add_jacobian(ci + 7, n3_i + k, -self.Jbb[k + 8, 3], limb_idx, l3)
                        self.stepper.add_jacobian(n3_i + k, ci + 7, -self.Jbb[7, k + 8], l3, limb_idx)

                    self.stepper.add_jacobian(ci + 3, ci + 3, -self.Jbb[3, 3], limb_idx)
                    self.stepper.add_jacobian(ci + 3, ci + 7, -self.Jbb[7, 3], limb_idx)
                    self.stepper.add_jacobian(ci + 7, ci + 3, -self.Jbb[3, 7], limb_idx)
                    self.stepper.add_jacobian(ci + 7, ci + 7, -self.Jbb[7, 7], limb_idx)

            limb_idx += 1
        print("HIHIHIHIHIHI")
        print(self.stepper)

        # Iterate through the joints
        joint_idx = 0
        num_limbs = len(self.soft_robots.limbs)
        for joint in self.soft_robots.joints:
            curr_iter = 0
            self.gradKappa1 = self.gradKappa1s[num_limbs + joint_idx]
            self.gradKappa2 = self.gradKappa2s[num_limbs + joint_idx]
            n2 = joint.joint_node
            l2 = joint.joint_limb

            for i in range(joint.ne):
                n1 = joint.connected_nodes[i][0]
                l1 = joint.connected_nodes[i][1]
                for j in range(i+1, joint.ne):
                    n3 = joint.connected_nodes[j][0]
                    l3 = joint.connected_nodes[j][1]

                    sgn1 = joint.sgns[curr_iter][0]
                    sgn2 = joint.sgns[curr_iter][1]
                    theta1_i = joint.theta_inds[curr_iter][0]
                    theta2_i = joint.theta_inds[curr_iter][1]

                    self.norm_e = joint.edge_len[i]
                    self.norm_f = joint.edge_len[j]
                    self.te = sgn1 * joint.tangents[i]
                    self.tf = sgn2 * joint.tangents[j]
                    self.d1e = joint.m1[curr_iter][0]
                    self.d2e = joint.m2[curr_iter][0]
                    self.d1f = joint.m1[curr_iter][1]
                    self.d2f = joint.m2[curr_iter][1]

                    self.norm2_e = self.norm_e ** 2
                    self.norm2_f = self.norm_f ** 2

                    self.chi = 1.0 + np.dot(self.te, self.tf)
                    self.tilde_t = (self.te + self.tf) / self.chi
                    self.tilde_d1 = (self.d1e + self.d1f) / self.chi
                    self.tilde_d2 = (self.d2e + self.d2f) / self.chi

                    self.kappa1 = joint.kappa[curr_iter, 0]
                    self.kappa2 = joint.kappa[curr_iter, 1]

                    self.kb_local = joint.kb[curr_iter]

                    # Compute Jacobians for joint
                    self.jacobian_computation()

                    # Adjust Jbb for joint-related cases
                    if sgn1 == -1:
                        self.Jbb[:, 3] *= -1
                        self.Jbb[3, :] *= -1
                    if sgn2 == -1:
                        self.Jbb[:, 7] *= -1
                        self.Jbb[7, :] *= -1

                    # NEED TO ADD THE NODAL FORCE CODE
                    for t in range(3):
                        for k in range(3):
                            self.stepper.add_jacobian(4*n1+t, 4*n1+k, -self.Jbb[k, t], l1)
                            self.stepper.add_jacobian(4*n1+t, 4*n2+k, -self.Jbb[k+4, t], l1, l2)
                            self.stepper.add_jacobian(4*n1+t, 4*n3+k, -self.Jbb[k+8, t], l1, l3)

                            self.stepper.add_jacobian(4*n2+t, 4*n1+k, -self.Jbb[k, t+4], l2, l1)
                            self.stepper.add_jacobian(4*n2+t, 4*n2+k, -self.Jbb[k+4, t+4], l2)
                            self.stepper.add_jacobian(4*n2+t, 4*n3+k, -self.Jbb[k+8, t+4], l2, l3)

                            self.stepper.add_jacobian(4*n3+t, 4*n1+k, -self.Jbb[k, t+8], l3, l1)
                            self.stepper.add_jacobian(4*n3+t, 4*n2+k, -self.Jbb[k+4, t+8], l3, l2)
                            self.stepper.add_jacobian(4*n3+t, 4*n3+k, -self.Jbb[k+8, t+8], l3)

                    if theta1_i is not None and theta2_i is not None:
                        # Add thetheta jacobians
                        for k in range(3):
                            self.stepper.add_jacobian(theta1_i, 4*n1+k, -self.Jbb[k, 3], l1)
                            self.stepper.add_jacobian(4*n1+k, theta1_i, -self.Jbb[3, k], l1)
                            self.stepper.add_jacobian(theta2_i, 4*n3+k, -self.Jbb[k+8, 7], l3)
                            self.stepper.add_jacobian(4*n3+k, theta2_i, -self.Jbb[7, k+8], l3)

                    curr_iter += 1

            joint_idx += 1
            
    def cross_mat(self, a):
        b = np.zeros((3, 3))
        b[0, 1] = -a[2]
        b[0, 2] = a[1]
        b[1, 0] = a[2]
        b[1, 2] = -a[0]
        b[2, 0] = -a[1]
        b[2, 1] = a[0]
        return b

    def jacobian_computation(self):
        self.tt_o_tt = np.outer(self.tilde_t, self.tilde_t)

        self.tilde_d1_3d = self.cross_mat(self.tilde_d1)
        self.tilde_d2_3d = self.cross_mat(self.tilde_d2)

        tmp = np.cross(self.tf, self.tilde_d2)
        self.tf_c_d2t_o_tt = np.outer(tmp, self.tilde_t)
        self.tt_o_tf_c_d2t = self.tf_c_d2t_o_tt.T
        self.kb_o_d2e = np.outer(self.kb_local, self.d2e.T)
        self.d2e_o_kb = self.kb_o_d2e.T

        self.D2kappa1De2 = (1.0 / self.norm2_e * (2 * self.kappa1 * self.tt_o_tt - self.tf_c_d2t_o_tt - self.tt_o_tf_c_d2t) 
                    - self.kappa1 / (self.chi * self.norm2_e) * (self.Id3 - np.outer(self.te, self.te))
                    + 1.0 / (4.0 * self.norm2_e) * (self.kb_o_d2e + self.d2e_o_kb))

        tmp = np.cross(self.te, self.tilde_d2)
        self.te_c_d2t_o_tt = np.outer(tmp, self.tilde_t)
        self.tt_o_te_c_d2t = self.te_c_d2t_o_tt.T
        self.kb_o_d2f = np.outer(self.kb_local, self.d2f.T)
        self.d2f_o_kb = self.kb_o_d2f.T

        self.D2kappa1Df2 = (1.0 / self.norm2_f * (2 * self.kappa1 * self.tt_o_tt + self.te_c_d2t_o_tt + self.tt_o_te_c_d2t)
                    - self.kappa1 / (self.chi * self.norm2_f) * (self.Id3 - np.outer(self.tf, self.tf))
                    + 1.0 / (4.0 * self.norm2_f) * (self.kb_o_d2f + self.d2f_o_kb))

        self.D2kappa1DeDf = (-self.kappa1 / (self.chi * self.norm_e * self.norm_f) * (self.Id3 + np.outer(self.te, self.tf))
                        + 1.0 / (self.norm_e * self.norm_f) * (2 * self.kappa1 * self.tt_o_tt - self.tf_c_d2t_o_tt 
                                                    + self.tt_o_te_c_d2t - self.tilde_d2_3d))
        self.D2kappa1DfDe = self.D2kappa1DeDf.T

        tmp = np.cross(self.tf, self.tilde_d1)
        self.tf_c_d1t_o_tt = np.outer(tmp, self.tilde_t)
        self.tt_o_tf_c_d1t = self.tf_c_d1t_o_tt.T
        self.kb_o_d1e = np.outer(self.kbLocal, self.d1e.T)
        self.d1e_o_kb = self.kb_o_d1e.T

        self.D2kappa2De2 = (1.0 / self.norm2_e * (2 * self.kappa2 * self.tt_o_tt + self.tf_c_d1t_o_tt + self.tt_o_tf_c_d1t)
                    - self.kappa2 / (self.chi * self.norm2_e) * (self.Id3 - np.outer(self.te, self.te))
                    - 1.0 / (4.0 * self.norm2_e) * (self.kb_o_d1e + self.d1e_o_kb))

        tmp = np.cross(self.te, self.tilde_d1)
        self.te_c_d1t_o_tt = np.outer(tmp, self.tilde_t)
        self.tt_o_te_c_d1t = self.te_c_d1t_o_tt.T
        self.kb_o_d1f = np.outer(self.kb_local, self.d1f.T)
        self.d1f_o_kb = self.kb_o_d1f.T

        self.D2kappa2Df2 = (1.0 / self.norm2_f * (2 * self.kappa2 * self.tt_o_tt - self.te_c_d1t_o_tt - self.tt_o_te_c_d1t)
                    - self.kappa2 / (self.chi * self.norm2_f) * (self.Id3 - np.outer(self.tf, self.tf))
                    - 1.0 / (4.0 * self.norm2_f) * (self.kb_o_d1f + self.d1f_o_kb))

        self.D2kappa2DeDf = (-self.kappa2 / (self.chi * self.norm_e * self.norm_f) * (self.Id3 + np.outer(self.te, self.tf))
                        + 1.0 / (self.norm_e * self.norm_f) * (2 * self.kappa2 * self.tt_o_tt + self.tf_c_d1t_o_tt
                                                    - self.tt_o_te_c_d1t + self.tilde_d1_3d))
        self.D2kappa2DfDe = self.D2kappa2DeDf.T

        self.D2kappa1Dthetae2 = -0.5 * self.kb_local.dot(self.d2e)
        self.D2kappa1Dthetaf2 = -0.5 * self.kb_local.dot(self.d2f)
        self.D2kappa2Dthetae2 = 0.5 * self.kb_local.dot(self.d1e)
        self.D2kappa2Dthetaf2 = 0.5 * self.kb_local.dot(self.d1f)

        self.D2kappa1DeDthetae = (1.0 / self.norm_e * ((0.5 * self.kb_local.dot(self.d1e)) * self.tilde_t
                                            - 1.0 / self.chi * np.cross(self.tf, self.d1e)))
        self.D2kappa1DeDthetaf = (1.0 / self.norm_e * ((0.5 * self.kb_local.dot(self.d1f)) * self.tilde_t
                                            - 1.0 / self.chi * np.cross(self.tf, self.d1f)))
        self.D2kappa1DfDthetae = (1.0 / self.norm_f * ((0.5 * self.kb_local.dot(self.d1e)) * self.tilde_t
                                            + 1.0 / self.chi * np.cross(self.te, self.d1e)))
        self.D2kappa1DfDthetaf = (1.0 / self.norm_f * ((0.5 * self.kb_local.dot(self.d1f)) * self.tilde_t
                                            + 1.0 / self.chi * np.cross(self.te, self.d1f)))
        self.D2kappa2DeDthetae = (1.0 / self.norm_e * ((0.5 * self.kb_local.dot(self.d2e)) * self.tilde_t
                                            - 1.0 / self.chi * np.cross(self.tf, self.d2e)))
        self.D2kappa2DeDthetaf = (1.0 / self.norm_e * ((0.5 * self.kb_local.dot(self.d2f)) * self.tilde_t
                                            - 1.0 / self.chi * np.cross(self.tf, self.d2f)))
        self.D2kappa2DfDthetae = (1.0 / self.norm_f * ((0.5 * self.kb_local.dot(self.d2e)) * self.tilde_t
                                            + 1.0 / self.chi * np.cross(self.te, self.d2e)))
        self.D2kappa2DfDthetaf = (1.0 / self.norm_f * ((0.5 * self.kb_local.dot(self.d2f)) * self.tilde_t
                                            + 1.0 / self.chi * np.cross(self.te, self.d2f)))

        # print(self.DDkappa1)
        self.DDkappa1 = np.zeros((11, 11))
        self.DDkappa1[0:3, 0:3] = self.D2kappa1De2
        # print("RETTET")
        # print(-self.D2kappa1De2 + self.D2kappa1DeDf)
        self.DDkappa1[0:3, 4:7] = -self.D2kappa1De2 + self.D2kappa1DeDf
        # print("D2Kappa1dedf")
        # print(-self.D2kappa1DeDf)
        # print("GETING THIG")
        # print(self.DDkappa1[0:3, 8:11])
        self.DDkappa1[0:3, 8:11] = -self.D2kappa1DeDf
        self.DDkappa1[4:7, 0:3] = -self.D2kappa1De2 + self.D2kappa1DfDe
        self.DDkappa1[4:7, 4:7] = self.D2kappa1De2 - self.D2kappa1DeDf - self.D2kappa1DfDe + self.D2kappa1Df2
        self.DDkappa1[4:7, 8:11] = self.D2kappa1DeDf - self.D2kappa1Df2
        self.DDkappa1[8:11, 0:3] = -self.D2kappa1DfDe
        self.DDkappa1[8:11, 4:7] = self.D2kappa1DfDe - self.D2kappa1Df2
        self.DDkappa1[8:11, 8:11] = self.D2kappa1Df2

        self.DDkappa1[3, 3] = self.D2kappa1Dthetae2
        self.DDkappa1[7, 7] = self.D2kappa1Dthetaf2

        self.DDkappa1[0:3, 3] = -self.D2kappa1DeDthetae
        self.DDkappa1[4:7, 3] = self.D2kappa1DeDthetae - self.D2kappa1DfDthetae
        self.DDkappa1[8:11, 3] = self.D2kappa1DfDthetae

        self.DDkappa1[0:3, 7] = -self.D2kappa1DeDthetaf
        self.DDkappa1[4:7, 7] = self.D2kappa1DeDthetaf - self.D2kappa1DfDthetaf
        self.DDkappa1[8:11, 7] = self.D2kappa1DfDthetaf

        self.DDkappa2 = np.zeros((11, 11))
        self.DDkappa2[0:3, 0:3] = self.D2kappa2De2
        self.DDkappa2[0:3, 4:7] = -self.D2kappa2De2 + self.D2kappa2DeDf
        self.DDkappa2[0:3, 8:11] = -self.D2kappa2DeDf
        self.DDkappa2[4:7, 0:3] = -self.D2kappa2De2 + self.D2kappa2DfDe
        self.DDkappa2[4:7, 4:7] = self.D2kappa2De2 - self.D2kappa2DeDf - self.D2kappa2DfDe + self.D2kappa2Df2
        self.DDkappa2[4:7, 8:11] = self.D2kappa2DeDf - self.D2kappa2Df2
        self.DDkappa2[8:11, 0:3] = -self.D2kappa2DfDe
        self.DDkappa2[8:11, 4:7] = self.D2kappa2DfDe - self.D2kappa2Df2
        self.DDkappa2[8:11, 8:11] = self.D2kappa2Df2

        self.DDkappa2[3, 3] = self.D2kappa2Dthetae2
        self.DDkappa2[7, 7] = self.D2kappa2Dthetaf2

        self.DDkappa2[0:3, 3] = -self.D2kappa2DeDthetae
        self.DDkappa2[4:7, 3] = self.D2kappa2DeDthetae - self.D2kappa2DfDthetae
        self.DDkappa2[8:11, 3] = self.D2kappa2DfDthetae

        self.DDkappa2[0:3, 7] = -self.D2kappa2DeDthetaf
        self.DDkappa2[4:7, 7] = self.D2kappa2DeDthetaf - self.D2kappa2DfDthetaf
        self.DDkappa2[8:11, 7] = self.D2kappa2DfDthetaf