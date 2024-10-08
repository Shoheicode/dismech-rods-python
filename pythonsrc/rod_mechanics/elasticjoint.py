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
        self.update_connected_nodes(node_num, limb_idx, True)
        self.setup()

    def update_joint(self) -> None:
        """
        Update the positions and velocities of the joint node.
        All positions and velocities are computed within ElasticRod.
        """
        if self.joint_limb >= len(self.limbs):
            return
            
        rod = self.limbs[self.joint_limb]
        self.x = rod.get_position(self.joint_node)
        self.u = rod.get_velocity(self.joint_node)

    def update_rods(self) -> None:
        """
        Update the positions of the "replaced" nodes in other limbs.
        """
        for node_num, limb_idx in self.replaced_nodes:
            if limb_idx >= len(self.limbs):
                continue
            rod = self.limbs[limb_idx]
            rod.set_position(node_num, self.x)
            rod.set_velocity(node_num, self.u)

    @staticmethod
    def rotate_axis_angle(v: np.ndarray, z: np.ndarray, theta: float) -> None:
        """
        Rotate vector v around axis z by angle theta (in-place).
        """
        c = np.cos(theta)
        s = np.sin(theta)
        v_rot = (c * v + 
                s * np.cross(z, v) + 
                np.dot(z, v) * (1 - c) * z)
        v[:] = v_rot

    @staticmethod
    def parallel_transport(d1_1: np.ndarray, t1: np.ndarray, t2: np.ndarray, 
                         d1_2: np.ndarray) -> None:
        """
        Parallel transport reference director d1_1 from t1 to t2.
        Result is stored in d1_2.
        """
        b = np.cross(t1, t2)
        if np.allclose(b, 0):
            d1_2[:] = d1_1
            return
            
        b = b / np.linalg.norm(b)
        theta = np.arccos(np.clip(np.dot(t1, t2), -1.0, 1.0))
        ElasticJoint.rotate_axis_angle(d1_1, b, theta)
        d1_2[:] = d1_1

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
        self.x0 = self.x.copy()
        self.u0 = self.u.copy()
        self.compute_time_parallel()
        
    def setup(self) -> None:
        """Set up the joint's physical properties."""
    
    def __set_mass(self):
        self.mass = 0
        pass

    def __set_reference_length(self):
        pass
    
    def __get_ref_twist(self):
        pass

    def __compute_twist_bar(self):
        pass

    def __compute_edge_len(self):
        pass

    def __compute_time_parallel(self):
        pass

    def __compute_kappa(self):
        pass

    def __compute_tangent(self):
        pass

    def __create_reference_directors(self):
        pass
    
    def __compute_material_directors(self):
        pass
    