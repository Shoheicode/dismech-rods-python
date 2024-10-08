import numpy as np
from typing import List, Tuple, Optional

class ElasticRod:
    EI: float
    def __init__(self, 
                 limb_idx: int,
                 start: np.ndarray = None,
                 end: np.ndarray = None,
                 num_nodes=None,
                 nodes: np.ndarray =None,
                 rho: float = None,
                 rod_radius: float = None,
                 youngs_modulus: float = None,
                 poisson_ratio: float = None,
                 mu: float = None):
        """
        Initialize an elastic rod with arbitrary shape defined by nodes.
        
        Args:
            limb_idx: The limb id of the rod. Should be a +1 increment of the previous rod.
            nodes: Array of shape (N, 3) representing consecutive nodes of the rod.
            rho: Density of the rod [kg/m^3].
            rod_radius: Radius of the rod [m].
            youngs_modulus: Young's modulus of the rod [N/m^2].
            poisson_ratio: Poisson's ratio of the rod.
            mu: Friction coefficient of the rod.
        """
        self.limb_idx = limb_idx
        if nodes == None:
            self.nv = num_nodes
            self.ne = num_nodes - 1
            self.ndof = num_nodes * 4 - 1
            self.rho = rho
            self.rod_radius = rod_radius
            self.youngM = youngs_modulus
            self.poisson_ratio = poisson_ratio
            self.mu = mu

            self.rod_length = np.linalg.norm(end - start)
            dir_vec = (end - start) / (num_nodes - 1)
            nodes = [start + i * dir_vec for i in range(num_nodes)]
            
        else:
            # Constructor with pre-defined nodes
            self.nv = len(nodes)
            self.ne = self.nv - 1
            self.ndof = len(nodes) * 4 - 1
            self.rho = rho
            self.rod_radius = rod_radius
            self.youngM = youngs_modulus
            self.poisson_ratio = poisson_ratio
            self.mu = mu

            self.rod_length = sum(np.linalg.norm(nodes[i] - nodes[i - 1]) for i in range(1, self.nv))
        
        # Initialize geometry
        self.setup(nodes)
        
        # Initialize state vectors
        self.x0 = np.zeros(self.ndof)  # Previous timestep DOFs
        self.x = np.zeros(self.ndof)   # Current timestep DOFs
        self.x_ls = np.zeros(self.ndof)  # Line search state
        self.u0 = np.zeros(self.ndof)  # Previous timestep velocities
        self.u = np.zeros(self.ndof)   # Current timestep velocities
        
        # Initialize constraint maps
        self.is_constrained = np.zeros(self.ndof, dtype=bool)
        self.is_dof_joint = np.zeros(self.ndof, dtype=int)
        self.is_node_joint = np.zeros(self.nv, dtype=int)
        self.is_edge_joint = np.zeros(self.ne, dtype=int)
        self.joint_ids: List[Tuple[int, int]] = []

    def setup(self, nodes: np.ndarray):
        """Setup basic rod geometry and allocate arrays."""
        self.nv = len(nodes)  # Number of vertices
        self.ne = self.nv - 1  # Number of edges
        self.ndof = 3 * self.nv + self.ne  # Total degrees of freedom
        
        # Initialize geometry arrays
        self.edge_len = np.zeros(self.ne)
        self.ref_len = np.zeros(self.ne)
        self.voronoi_len = np.zeros(self.nv)
        
        # Initialize frames
        self.d1 = np.zeros((self.ne, 3))
        self.d2 = np.zeros((self.ne, 3))
        self.d1_old = np.zeros((self.ne, 3))
        self.d2_old = np.zeros((self.ne, 3))
        self.m1 = np.zeros((self.ne, 3))
        self.m2 = np.zeros((self.ne, 3))
        self.tangent = np.zeros((self.ne, 3))
        self.tangent_old = np.zeros((self.ne, 3))
        
        # Initialize twists and curvatures
        self.ref_twist = np.zeros(self.ne)
        self.ref_twist_old = np.zeros(self.ne)
        self.twist_bar = np.zeros(self.ne)
        self.kappa = np.zeros((self.nv, 2))
        self.kappa_bar = np.zeros((self.nv, 2))
        
        # Compute initial rod length
        self.rod_length = 0
        for i in range(self.ne):
            self.rod_length += np.linalg.norm(nodes[i+1] - nodes[i])

    def compute_elastic_stiffness(self):
        """Compute elastic stiffness parameters."""
        self.shearM = self.youngM / (2 * (1 + self.poisson_ratio))
        self.EA = self.youngM * self.cross_sectional_area
        self.EI = self.youngM * np.pi * self.rod_radius**4 / 4
        self.GJ = self.shearM * np.pi * self.rod_radius**4 / 2

    def prepare_for_iteration(self):
        """Update discrete values and frames for the next timestep."""
        pass

    def update_newton_x(self, dx: np.ndarray, offset: int, alpha: float = 1.0) -> float:
        """
        Perform Newton update iteration.
        
        Args:
            dx: Update vector from solver
            offset: Global system DOF offset
            alpha: Newton update coefficient
            
        Returns:
            Maximum non-theta update magnitude
        """
        max_update = 0.0
        for i in range(self.ndof):
            if not self.is_constrained[i]:
                update = alpha * dx[offset + self.full_to_uncons_map[i]]
                self.x[i] += update
                if i < 3 * self.nv:  # Non-theta DOF
                    max_update = max(max_update, abs(update))
        return max_update

    def updateGuess(self, weight:float, dt:float):
        pass

    def enable_2d_sim(self):
        """Enable 2D simulation in x-z plane."""
        for i in range(self.nv):
            # Constrain y position
            self.is_constrained[3*i + 1] = True
            # Constrain theta
            if i < self.ne:
                self.is_constrained[3*self.nv + i] = True
        self.update_map()
    
    def update_map(self):
        pass

    def add_joint(self, node_num: int, attach_to_joint: bool, joint_node: int, joint_limb: int ):
        pass

    def set_vertex_boundary_condition(position: np.ndarray, k: int):
        pass

    def set_theta_boundary_condition(desired_theta: float, k: int):
        pass

    def free_vertex_boundary_condition(k: int):
        pass

    def get_tangent(k: int):
        pass
    
    def get_theta(k: int):
        pass
    
    def __compute_elastic_stiffness(self):
        pass
    def __set_mass(self):
        pass
    def __set_reference_length(self):
        pass

    def __compute_tangent(self):
        pass
    def __compute_time_parallel(self):
        pass
    def __compute_material_director(self):
        pass
    def __compute_edge_len(self):
        pass
    def __get_ref_twist(self):
        pass
    def __compute_kappa(self):
        pass

    @staticmethod
    def __parallel_transport(self, d1_1: np.ndarray, t1: np.ndarray, t2: np.ndarray, 
                         d1_2: np.ndarray):
        """
        Parallel transport reference frame between edges.
        
        Args:
            d1_1: First director of first frame
            t1: First tangent
            t2: Second tangent
            d1_2: Output first director of second frame
        """
        b = np.cross(t1, t2)
        if np.allclose(b, 0):
            d1_2[:] = d1_1
            return
            
        b = b / np.linalg.norm(b)
        n1 = np.cross(t1, b)
        n2 = np.cross(t2, b)
        
        d1_2[:] = (np.dot(d1_1, n1) * n2 + np.dot(d1_1, b) * b)

    @staticmethod
    def __rotate_axis_angle(self, v: np.ndarray, z: np.ndarray, theta: float):
        """
        Rotate vector v around axis z by angle theta.
        
        Args:
            v: Vector to rotate
            z: Rotation axis
            theta: Rotation angle
        """
        c = np.cos(theta)
        s = np.sin(theta)
        v[:] = v * c + np.cross(z, v) * s + z * np.dot(z, v) * (1 - c)

    @staticmethod
    def __signed_angle(self, u: np.ndarray,v: np.ndarray, z: np.ndarray):
        pass