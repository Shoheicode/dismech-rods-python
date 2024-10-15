import numpy as np
from typing import List
from pythonFolder.pythonsrc.rod_mechanics.base_force import BaseForce
from soft_robots import SoftRobots

class ElasticTwistingForce(BaseForce):
    def __init__(self, m_soft_robots: List[SoftRobots]):
        super().__init__(m_soft_robots)
        self.ci = 0
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
        self.grad_twist_local = np.zeros(0)
        self.get_undeformed_twist = np.zeros(0)
        self.f = np.zeros(0)
        self.grad_twists: List[List[np.ndarray]] = []
        self.deltams: List[List[np.ndarray]] = []
        self.theta_fs: List[List[np.ndarray]] = []
        self.theta_es: List[List[np.ndarray]] = []
        self.d2m_de2 = self.d2m_df2 = self.d2m_de_df = self.d2m_df_de = np.zeros((3, 3))
        self.te_matrix = np.zeros((3, 3))
        self.j = np.zeros((11, 11))
        self.dd_twist = np.zeros((11, 11))
        self.jtt = np.zeros((11, 11))
        self.grad_twist: List[np.ndarray] = None
        self.gj = 0.0

    def compute_force(self, dt: float):
        for limb_idx, limb in enumerate(self.soft_robots.limbs):
            self.gj = limb.GJ
            self.grad_twist = self.grad_twists[limb_idx]
            self.deltam = self.deltams[limb_idx]
            self.theta_f = self.theta_fs[limb_idx]
            self.theta_e = self.theta_es[limb_idx]

            for i in range(limb.ne):
                self.theta_f[i] = limb.x[4*i + 3]

            for i in range(limb.ne):
                self.theta_e[i] = 0 if i == 0 else theta_f[i-1]

            deltam[:] = theta_f - theta_e

            for i in range(1, limb.ne):
                norm_e = limb.edge_len[i-1]
                norm_f = limb.edge_len[i]
                grad_twist[i, :3] = -0.5 / norm_e * limb.kb[i]
                grad_twist[i, 8:11] = 0.5 / norm_f * limb.kb[i]
                grad_twist[i, 4:7] = -(grad_twist[i, :3] + grad_twist[i, 8:11])
                grad_twist[i, 3] = -1
                grad_twist[i, 7] = 1

            for i in range(1, limb.ne):
                value = gj / limb.voronoi_len[i] * (deltam[i] + limb.ref_twist[i] - limb.twist_bar[i])
                f = -value * grad_twist[i]

                if limb.is_node_joint[i-1] != 1 and limb.is_node_joint[i] != 1 and limb.is_node_joint[i+1] != 1:
                    ci = 4*i - 4
                    for k in range(11):
                        ind = ci + k
                        self.stepper.add_force(ind, -f[k], limb_idx)
                else:
                    n1, l1 = limb.joint_ids[i-1]
                    n2, l2 = limb.joint_ids[i]
                    n3, l3 = limb.joint_ids[i+1]

                    for k in range(3):
                        self.stepper.add_force(4*n1+k, -f[k], l1)
                        self.stepper.add_force(4*n2+k, -f[k+4], l2)
                        self.stepper.add_force(4*n3+k, -f[k+8], l3)
                    
                    ci = 4*i - 4
                    self.stepper.add_force(ci+3, -f[3], limb_idx)
                    self.stepper.add_force(ci+7, -f[7], limb_idx)

        # Joint force computation
        for joint_idx, joint in enumerate(self.soft_robots.joints):
            gj = self.soft_robots.limbs[0].GJ  # NOTE: Change this later
            grad_twist = self.grad_twists[len(self.soft_robots.limbs) + joint_idx]
            deltam = self.deltams[len(self.soft_robots.limbs) + joint_idx]
            theta_f = self.theta_fs[len(self.soft_robots.limbs) + joint_idx]
            theta_e = self.theta_es[len(self.soft_robots.limbs) + joint_idx]

            curr_iter = 0
            for i in range(joint.ne):
                l1 = joint.connected_nodes[i][1]
                for j in range(i+1, joint.ne):
                    l3 = joint.connected_nodes[j][1]

                    sgn1, sgn2 = joint.sgns[curr_iter]
                    theta1_i, theta2_i = joint.theta_inds[curr_iter]

                    theta_e[curr_iter] = self.soft_robots.limbs[l1].x[theta1_i] * sgn1
                    theta_f[curr_iter] = self.soft_robots.limbs[l3].x[theta2_i] * sgn2

                    curr_iter += 1

            deltam[:] = theta_f - theta_e

            curr_iter = 0
            for i in range(joint.ne):
                for j in range(i+1, joint.ne):
                    norm_e = joint.edge_len[i]
                    norm_f = joint.edge_len[j]
                    grad_twist[curr_iter, :3] = -0.5 / norm_e * joint.kb[curr_iter]
                    grad_twist[curr_iter, 8:11] = 0.5 / norm_f * joint.kb[curr_iter]
                    grad_twist[curr_iter, 4:7] = -(grad_twist[curr_iter, :3] + grad_twist[curr_iter, 8:11])
                    grad_twist[curr_iter, 3] = -1
                    grad_twist[curr_iter, 7] = 1
                    curr_iter += 1

            curr_iter = 0
            n2, l2 = joint.joint_node, joint.joint_limb
            for i in range(joint.ne):
                n1, l1 = joint.connected_nodes[i]
                for j in range(i+1, joint.ne):
                    n3, l3 = joint.connected_nodes[j]

                    sgn1, sgn2 = joint.sgns[curr_iter]
                    theta1_i, theta2_i = joint.theta_inds[curr_iter]

                    value = gj / joint.voronoi_len[curr_iter] * (deltam[curr_iter] + 
                            joint.ref_twist[curr_iter] - joint.twist_bar[curr_iter])
                    f = -value * grad_twist[curr_iter]

                    for k in range(3):
                        self.stepper.add_force(4*n1+k, -f[k], l1)
                        self.stepper.add_force(4*n2+k, -f[k+4], l2)
                        self.stepper.add_force(4*n3+k, -f[k+8], l3)
                    
                    self.stepper.add_force(theta1_i, -f[3] * sgn1, l1)
                    self.stepper.add_force(theta2_i, -f[7] * sgn2, l3)

                    curr_iter += 1

    def compute_force_and_jacobian(self, dt: float):
        pass

    @staticmethod
    def cross_mat(a: np.ndarray, b: np.ndarray):
        return np.array([
            [0, -a[2], a[1]],
            [a[2], 0, -a[0]],
            [-a[1], a[0], 0]
        ])