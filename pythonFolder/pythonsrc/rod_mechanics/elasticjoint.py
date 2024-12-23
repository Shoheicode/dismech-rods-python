from typing import List, Tuple, Optional
import numpy as np
from dataclasses import dataclass
from numpy.typing import NDArray
from typing import List, Tuple, Optional

from pythonsrc.rod_mechanics.elastic_rod import ElasticRod

class ElasticJoint:
    def __init__(self, node: int, limb_idx: int, limbs: List[ElasticRod]):
        """
        Initialize an elastic joint.

        Args:
            node: The node on the elastic rod that will be converted to a joint
            limb_idx: The limb id of the rod
            limbs: Container of limbs
        """
        self.joint_node = node
        self.joint_limb = limb_idx
        self.ne = 0  # Total number of edges connected to this joint
        self.limbs = limbs

        self.limbs[limb_idx].add_joint(self.joint_node, False, 0,0)
        self.update_connected_nodes(self.joint_node, self.joint_limb, False)
        

        # Position and velocity vectors
        self.x0 = np.zeros(3)  # Previous timestep position
        self.x = np.zeros(3)   # Current timestep position
        self.x_ls = np.zeros(3)  # Position for line search
        self.u0 = np.zeros(3)  # Previous timestep velocity
        self.u = np.zeros(3)   # Current timestep velocity

        # Node connections
        self.connected_nodes: List[Tuple[int, int]] = []  # (node_id, limb_id)
        self.replaced_nodes: List[Tuple[int, int]] = []   # (node_id, limb_id)
        self.bending_twist_signs: List[int] = []
        
        # Physical properties
        self.num_bending_combos = 0
        self.sgns: List[np.ndarray] = []  # List of 2D vectors with values 1 or -1
        self.theta_inds: List[np.ndarray] = []  # List of 2D vectors with theta indices
        self.ref_len = np.array([])
        self.voronoi_len = np.array([])
        self.mass = 0.0

        # Geometric properties
        self.tangents = np.array([])
        self.tangents_old = np.array([])
        self.d1: List[np.ndarray] = []
        self.d2: List[np.ndarray] = []
        self.d1_old: List[np.ndarray] = []
        self.d2_old: List[np.ndarray] = []
        self.m1: List[np.ndarray] = []
        self.m2: List[np.ndarray] = []
        
        # Physical quantities
        self.ref_twist = np.array([])
        self.kb = np.array([])
        self.kappa = np.array([])
        self.kappa_bar = np.array([])
        self.twist_bar = np.array([])
        self.ref_twist_old = np.array([])
        self.edge_len = np.array([])

    def add_to_joint(self, node_num: int, limb_idx: int) -> None:
        """
        Add a connection to the joint.
        
        When we connect a node to a joint, that node is actually "replaced" with the joint node.
        We assume that nodes connected to a joint have positional overlap.

        Args:
            node_num: The id of the node being replaced by the joint node
            limb_idx: The limb id of the replaced node
        """
        self.limbs[limb_idx].add_joint(node_num, True, self.joint_node, self.joint_limb)
        self.update_connected_nodes(node_num, limb_idx, True)

    def update_joint(self) -> None:
        """
        Update the positions and velocities of the joint node.
        All positions and velocities are computed within ElasticRod.
        """
        self.x = np.zeros(3)
        self.u = np.zeros(3)
        self.x0 = np.zeros(3)
        self.u0 = np.zeros(3)

        self.x[0:3] = self.limbs[self.joint_limb].x[4 * self.joint_node:4 * self.joint_node + 3]
        self.x0[0:3] = self.limbs[self.joint_limb].x0[4 * self.joint_node:4 * self.joint_node + 3]
        self.u[0:3] = self.limbs[self.joint_limb].u[4 * self.joint_node:4 * self.joint_node + 3]
        self.u0[0:3] = self.limbs[self.joint_limb].u0[4 * self.joint_node:4 * self.joint_node + 3]


    def update_rods(self) -> None:
        """
        Update the positions of the "replaced" nodes in other limbs.
        """
        for num_node, limb_idx in self.replaced_nodes:
            curr_limb = self.limbs[limb_idx]
            curr_limb.x[4 * num_node:4 * num_node + 3] = self.x[0:3]

    def update_connected_nodes(self, node_num: int, limb_idx: int, remove_dof: bool):
        nv = self.limbs[limb_idx].nv
        if node_num == 0:
            self.connected_nodes.append((1, limb_idx))
            self.bending_twist_signs.append(-1)
            if remove_dof:
                self.replaced_nodes.append((0, limb_idx))
            self.ne += 1
        elif node_num == nv - 1:
            self.connected_nodes.append((nv - 2, limb_idx))
            self.bending_twist_signs.append(1)
            if remove_dof:
                self.replaced_nodes.append((nv - 1, limb_idx))
            self.ne += 1
        else:
            self.connected_nodes.append((node_num - 1, limb_idx))
            self.bending_twist_signs.append(1)
            self.connected_nodes.append((node_num + 1, limb_idx))
            self.bending_twist_signs.append(-1)
            if remove_dof:
                self.replaced_nodes.append((node_num, limb_idx))
            self.ne += 2


    @staticmethod
    def rotate_axis_angle(v: np.ndarray, z: np.ndarray, theta: float) -> None:
        """
        Rotate vector v around axis z by angle theta (in-place).
        """
        if theta != 0:
            cs, ss = np.cos(theta), np.sin(theta)
            v[:] = cs * v + ss * np.cross(z, v) + np.dot(z, v) * (1.0 - cs) * z

        return v[:]

    @staticmethod
    def parallel_transport(d1_1: np.ndarray, t1: np.ndarray, t2: np.ndarray, 
                         d1_2: np.ndarray) -> None:
        """
        Parallel transport reference director d1_1 from t1 to t2.
        Result is stored in d1_2.
        """
        b = np.cross(t1,t2)
        if (np.linalg.norm(b) == 0):
            d1_2 = d1_1
        else:
            b = b / np.linalg.norm(b)
            # The following four lines may seem unnecessary but can sometimes help
            # with numerical stability
            b = b - np.dot(b,t1) * t1
            b = b / np.linalg.norm(b)
            b = b - np.dot(b,t2) * t2
            b = b / np.linalg.norm(b)
            n1 = np.cross(t1,b)
            n2 = np.cross(t2,b)
            d1_2 = np.dot(d1_1,t1) * t2 + np.dot(d1_1,n1) * n2 + np.dot(d1_1,b) * b

        return d1_2

    @staticmethod
    def signed_angle(u: np.ndarray, v: np.ndarray, n: np.ndarray) -> float:
        """
        Compute signed angle between vectors u and v with respect to normal n.
        """
        w = np.cross(u, v)
        angle = np.arctan2(np.linalg.norm(w), np.dot(u, v))
        return angle if np.dot(w, n) >= 0 else -angle

    def prep_limbs(self) -> None:
        """Prepare limbs for computation."""
        self.update_joint()
        self.update_rods()
        
    def prepare_for_iteration(self) -> None:
        """Prepare joint for next iteration."""
        self.__compute_tangent()
        self.__compute_time_parallel()
        self.__compute_material_directors()
        self.__get_ref_twist()
        self.__compute_edge_len()
        self.__compute_kappa()
        
    def setup(self) -> None:
        self.num_bending_combos = 0
        for i in range(self.ne):
            n1 = self.connected_nodes[i][0]
            sgn1 = 1 if self.bending_twist_signs[i] == 1 else -1
            theta1_i = 4 * n1 + 3 if self.bending_twist_signs[i] == 1 else 4 * n1 - 1
            for j in range(i + 1, self.ne):
                n2 = self.connected_nodes[j][0]
                sgn2 = -1 if self.bending_twist_signs[j] == 1 else 1
                theta2_i = 4 * n2 + 3 if self.bending_twist_signs[j] == 1 else 4 * n2 - 1
                self.sgns.append((sgn1, sgn2))
                self.theta_inds.append((theta1_i, theta2_i))
                self.num_bending_combos += 1

        self.update_joint()
        self.update_rods()
        self.x0 = self.x.copy()
        self.u0 = self.u.copy()

        self.ref_len = np.zeros(self.ne)
        self.voronoi_len = np.zeros(self.num_bending_combos)

        self.tangents = np.zeros((self.ne, 3))

        for _ in range(self.num_bending_combos):
            self.d1.append(np.zeros((2, 3)))
            self.d2.append(np.zeros((2, 3)))
            self.d1_old.append(np.zeros((2, 3)))
            self.d2_old.append(np.zeros((2, 3)))
            self.m1.append(np.zeros((2, 3)))
            self.m2.append(np.zeros((2, 3)))

        self.ref_twist = np.zeros(self.num_bending_combos)
        self.ref_twist_old = np.zeros(self.num_bending_combos)

        self.kb = np.zeros((self.num_bending_combos, 3))
        self.kappa = np.zeros((self.num_bending_combos, 2))
        self.twist_bar = np.zeros(self.num_bending_combos)
        self.edge_len = np.zeros(self.ne)

        self.__set_reference_length()
        self.__set_mass()
        self.__compute_tangent()
        self.__create_reference_directors()
        self.__compute_material_directors()
        self.__compute_kappa()

        self.kappa_bar = self.kappa.copy()
        self.__get_ref_twist()
        self.__compute_twist_bar()
        self.__compute_edge_len()

        self.d1_old = self.d1.copy()
        self.d2_old = self.d2.copy()
        self.tangents_old = self.tangents.copy()
        self.ref_twist_old = self.ref_twist.copy()
    
    def __set_mass(self):
        self.mass = 0
        for i in range(self.ne):
            limb_idx = self.connected_nodes[i][1]
            curr_limb = self.limbs[limb_idx]
            self.mass += 0.5 * self.ref_len[i] * curr_limb.cross_sectional_area * curr_limb.rho

    def __set_reference_length(self):
        for i in range(self.ne):
            node = self.connected_nodes[i][0]
            limb_idx = self.connected_nodes[i][1]
            curr_limb = self.limbs[limb_idx]
            self.ref_len[i] = np.linalg.norm(self.x[0:3] - curr_limb.x[4 * node:4 * node + 3])

        curr_index = 0
        for i in range(self.ne):
            for j in range(i + 1, self.ne):
                self.voronoi_len[curr_index] = 0.5 * (self.ref_len[i] + self.ref_len[j])
                curr_index += 1
    
    def __get_ref_twist(self):
        for i in range(self.ne):
            for j in range(i + 1, self.ne):
                sgn1 = self.sgns[i][0]
                sgn2 = self.sgns[i][1]
                u0 = self.d1[i][0]
                u1 = self.d1[i][1]
                t0 = self.tangents[i] * sgn1
                t1 = self.tangents[j] * sgn2
                ut = np.zeros(3)
                self.parallel_transport(u0, t0, t1, ut)
                self.rotate_axis_angle(ut, t1, self.ref_twist_old[i])

                sgn_angle = self.signed_angle(ut, u1, t1)
                self.ref_twist[i] = self.ref_twist_old[i] + sgn_angle

    def __compute_twist_bar(self):
        for i in range(self.ne):
            for j in range(i + 1, self.ne):
                theta_i = self.limbs[self.connected_nodes[i][1]].x[self.theta_inds[i][0]] * self.sgns[i][0]
                theta_f = self.limbs[self.connected_nodes[j][1]].x[self.theta_inds[j][1]] * self.sgns[i][1]
                self.twist_bar[i] = theta_f - theta_i + self.ref_twist[i]

    def __compute_edge_len(self):
        for i in range(self.ne):
            num_node = self.connected_nodes[i][0]
            limb_idx = self.connected_nodes[i][1]
            curr_limb = self.limbs[limb_idx]
            self.edge_len[i] = np.linalg.norm(self.x[0:3] - curr_limb.x[4 * num_node:4 * num_node + 3])

    def __compute_time_parallel(self):
        for i in range(self.ne):
            for j in range(i + 1, self.ne):
                sgn1 = self.sgns[i][0]
                sgn2 = self.sgns[i][1]
                t0_0 = self.tangents_old[i] * sgn1
                t1_0 = self.tangents[i] * sgn1
                t0_1 = self.tangents_old[j] * sgn2
                t1_1 = self.tangents[j] * sgn2

                d1_0 = np.zeros(3)
                d1_1 = np.zeros(3)
                self.parallel_transport(self.d1_old[i][0], t0_0, t1_0, d1_0)
                self.parallel_transport(self.d1_old[i][1], t0_1, t1_1, d1_1)

                self.d1[i][0] = d1_0
                self.d2[i][0] = np.cross(t1_0, d1_0)
                self.d1[i][1] = d1_1
                self.d2[i][1] = np.cross(t1_1, d1_1)

    def __compute_kappa(self):
        for i in range(self.ne):
            for j in range(i + 1, self.ne):
                sgn1 = self.sgns[i][0]
                sgn2 = self.sgns[i][1]
                t0 = self.tangents[i] * sgn1
                t1 = self.tangents[j] * sgn2
                self.kb[i] = 2.0 * np.cross(t0, t1) / (1.0 + np.dot(t0, t1))

    def __compute_tangent(self):
        for i in range(self.ne):
            node_num = self.connected_nodes[i][0]
            limb_idx = self.connected_nodes[i][1]
            curr_limb = self.limbs[limb_idx]

            if self.bending_twist_signs[i] == 1:
                self.tangents[i] = self.x - curr_limb.x[4 * node_num:4 * node_num + 3]
            else:
                self.tangents[i] = curr_limb.x[4 * node_num:4 * node_num + 3] - self.x
            self.tangents[i] /= np.linalg.norm(self.tangents[i])

    def __create_reference_directors(self):
        for i in range(self.ne):
            for j in range(i + 1, self.ne):
                sgn1 = self.sgns[i][0]
                sgn2 = self.sgns[i][1]
                t0 = self.tangents[i] * sgn1
                t1 = self.tangents[j] * sgn2
                tmp1 = np.array([0, 0, -1])
                tmp2 = np.cross(t0, tmp1)

                if np.linalg.norm(tmp2) < 1.0e-6:
                    tmp1 = np.array([0, 1, 0])
                    tmp2 = np.cross(t0, tmp1)

                tmp2 /= np.linalg.norm(tmp2)
                self.d1[i][0] = tmp2
                self.d2[i][0] = np.cross(t0, tmp2)

                tmp1 = self.parallel_transport(tmp2, t0, t1, tmp1)
                self.d1[i][1] = tmp1
                self.d2[i][1] = np.cross(t1, tmp1)
    
    def __compute_material_directors(self):
        for i in range(self.ne):
            for j in range(i + 1, self.ne):
                sgn1 = self.sgns[i][0]
                sgn2 = self.sgns[i][1]
                theta1_i = self.theta_inds[i][0]
                theta2_i = self.theta_inds[i][1]

                limb1 = self.limbs[self.connected_nodes[i][1]]
                limb2 = self.limbs[self.connected_nodes[j][1]]
                angle1 = limb1.x[theta1_i] * sgn1
                angle2 = limb2.x[theta2_i] * sgn2
                cs1, ss1 = np.cos(angle1), np.sin(angle1)
                cs2, ss2 = np.cos(angle2), np.sin(angle2)

                self.m1[i][0] = cs1 * self.d1[i][0] + ss1 * self.d2[i][0]
                self.m2[i][0] = -ss1 * self.d1[i][0] + cs1 * self.d2[i][0]

                self.m1[i][1] = cs2 * self.d1[i][1] + ss2 * self.d2[i][1]
                self.m2[i][1] = -ss2 * self.d1[i][1] + cs2 * self.d2[i][1]
    