from typing import List
from pythonsrc.rod_mechanics.base_force import BaseForce
from pythonsrc.rod_mechanics.elastic_rod import ElasticRod
from pythonsrc.rod_mechanics.soft_robots import SoftRobots
import numpy as np

class elasticBendingForce(BaseForce):
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
    kappa11 = np.zeros((3, 3))
    kappa22 = np.zeros((3, 3))
    f = np.zeros(3)

    # Lists of matrices (using numpy arrays instead of Eigen::MatrixXd)
    gradKappa1s: List[np.ndarray] = []
    gradKappa2s: List[np.ndarray] = []

    gradKappa1 = np.zeros((3, 3))
    gradKappa2 = np.zeros((3, 3))
    relevantPart = np.zeros((3, 3))
    kappa = np.zeros((3, 3))
    DDkappa1 = np.zeros((3, 3))
    DDkappa2 = np.zeros((3, 3))
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

        for limb in soft_robots.limbs:
            EI = limb.EI
            print(EI)


    def compute_force(self, dt):
        limb_idx = 0

        for limb in super().soft_robots.limbs:
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

                self.chi = 1.0 + np.dot(te, tf)
                self.tilde_t = (te + tf) / chi
                self.tilde_d1 = (d1e + d1f) / chi
                self.tilde_d2 = (d2e + d2f) / chi

                self.kappa1 = limb.kappa[i, 0]
                self.kappa2 = limb.kappa[i, 1]

                self.Dkappa1De = (1.0 / norm_e) * (-kappa1 * tilde_t + np.cross(tf, tilde_d2))
                self.Dkappa1Df = (1.0 / norm_f) * (-kappa1 * tilde_t - np.cross(te, tilde_d2))
                self.Dkappa2De = (1.0 / norm_e) * (-kappa2 * tilde_t - np.cross(tf, tilde_d1))
                self.Dkappa2Df = (1.0 / norm_f) * (-kappa2 * tilde_t + np.cross(te, tilde_d1))

                self.gradKappa1[i, :3] = -Dkappa1De
                self.gradKappa1[i, 4:7] = Dkappa1De - Dkappa1Df
                self.gradKappa1[i, 8:11] = Dkappa1Df

                self.gradKappa2[i, :3] = -Dkappa2De
                self.gradKappa2[i, 4:7] = Dkappa2De - Dkappa2Df
                self.gradKappa2[i, 8:11] = Dkappa2Df

                self.kbLocal = limb.kb[i, :]

                self.gradKappa1[i, 3] = -0.5 * np.dot(kbLocal, d1e)
                self.gradKappa1[i, 7] = -0.5 * np.dot(kbLocal, d1f)
                self.gradKappa2[i, 3] = -0.5 * np.dot(kbLocal, d2e)
                self.gradKappa2[i, 7] = -0.5 * np.dot(kbLocal, d2f)

            # Second loop
            for i in range(1, limb.ne):
                self.relevantPart = np.zeros((11, 2))
                self.relevantPart[:, 0] = gradKappa1[i, :]
                self.relevantPart[:, 1] = gradKappa2[i, :]
                self.kappaL = limb.kappa[i, :] - limb.kappa_bar[i, :]

                self.f = -np.dot(self.relevantPart, np.dot(self.EIMatrices[limb_idx], self.kappaL)) / limb.voronoi_len[i]

                if not (limb.isNodeJoint[i - 1] or limb.isNodeJoint[i] or limb.isNodeJoint[i + 1]):
                    ci = 4 * i - 4
                    for k in range(11):
                        ind = ci + k
                        super().stepper.addForce(ind, -f[k], limb_idx)
                else:
                    n1, l1 = limb.joint_ids[i - 1]
                    n2, l2 = limb.joint_ids[i]
                    n3, l3 = limb.joint_ids[i + 1]

                    for k in range(3):
                        super().stepper.addForce(4 * n1 + k, -f[k], l1)
                        super().stepper.addForce(4 * n2 + k, -f[k + 4], l2)
                        super().stepper.addForce(4 * n3 + k, -f[k + 8], l3)

                    ci = 4 * i - 4
                    super().stepper.addForce(ci + 3, -f[3], limb_idx)
                    super().stepper.addForce(ci + 7, -f[7], limb_idx)

            limb_idx += 1

        joint_idx = 0
        num_limbs = len(super().soft_robots.limbs)

        for joint in super().soft_robots.joints:
            curr_iter = 0
            gradKappa1 = gradKappa1s[num_limbs + joint_idx]
            gradKappa2 = gradKappa2s[num_limbs + joint_idx]

            for i in range(joint.ne):
                sgn1 = 1 if joint.bending_twist_signs[i] == 1 else -1
                for j in range(i + 1, joint.ne):
                    sgn2 = -1 if joint.bending_twist_signs[j] == 1 else 1
                    norm_e = joint.edge_len[i]
                    norm_f = joint.edge_len[j]
                    te = sgn1 * joint.tangents[i, :]
                    tf = sgn2 * joint.tangents[j, :]
                    d1e = joint.m1[curr_iter, 0, :]
                    d2e = joint.m2[curr_iter, 0, :]
                    d1f = joint.m1[curr_iter, 1, :]
                    d2f = joint.m2[curr_iter, 1, :]

                    chi = 1.0 + np.dot(te, tf)
                    tilde_t = (te + tf) / chi
                    tilde_d1 = (d1e + d1f) / chi
                    tilde_d2 = (d2e + d2f) / chi

                    kappa1 = joint.kappa[curr_iter, 0]
                    kappa2 = joint.kappa[curr_iter, 1]

                    Dkappa1De = (1.0 / norm_e) * (-kappa1 * tilde_t + np.cross(tf, tilde_d2))
                    Dkappa1Df = (1.0 / norm_f) * (-kappa1 * tilde_t - np.cross(te, tilde_d2))
                    Dkappa2De = (1.0 / norm_e) * (-kappa2 * tilde_t - np.cross(tf, tilde_d1))
                    Dkappa2Df = (1.0 / norm_f) * (-kappa2 * tilde_t + np.cross(te, tilde_d1))

                    gradKappa1[curr_iter, :3] = -Dkappa1De
                    gradKappa1[curr_iter, 4:7] = Dkappa1De - Dkappa1Df
                    gradKappa1[curr_iter, 8:11] = Dkappa1Df

                    gradKappa2[curr_iter, :3] = -Dkappa2De
                    gradKappa2[curr_iter, 4:7] = Dkappa2De - Dkappa2Df
                    gradKappa2[curr_iter, 8:11] = Dkappa2Df

                    kbLocal = joint.kb[curr_iter, :]

                    gradKappa1[curr_iter, 3] = -0.5 * np.dot(kbLocal, d1e)
                    gradKappa1[curr_iter, 7] = -0.5 * np.dot(kbLocal, d1f)
                    gradKappa2[curr_iter, 3] = -0.5 * np.dot(kbLocal, d2e)
                    gradKappa2[curr_iter, 7] = -0.5 * np.dot(kbLocal, d2f)

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

                    relevantPart = np.zeros((11, 2))
                    relevantPart[:, 0] = gradKappa1[curr_iter, :]
                    relevantPart[:, 1] = gradKappa2[curr_iter, :]
                    kappaL = joint.kappa[curr_iter, :] - joint.kappa_bar[curr_iter, :]

                    f = -np.dot(relevantPart, np.dot(EIMatrices[0], kappaL)) / joint.voronoi_len[curr_iter]

                    for k in range(3):
                        stepper.addForce(4 * n1 + k, -f[k], l1)
                        stepper.addForce(4 * n2 + k, -f[k + 4], l2)
                        stepper.addForce(4 * n3 + k, -f[k + 8], l3)

                    stepper.addForce(theta1_i, -f[3] * sgn1, l1)
                    stepper.addForce(theta2_i, -f[7] * sgn2, l3)

                    curr_iter += 1

            joint_idx += 1
    
    def compute_force_and_jacobian(self, dt):
        return super().compute_force_and_jacobian(dt)
