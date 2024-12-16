import numpy as np
from typing import List, Tuple, Optional
import math

class ElasticRod:
    EI: float
    def __init__(self, 
                 limb_idx: int,
                 start: np.ndarray = None,
                 end: np.ndarray = None,
                 num_nodes: int =None,
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
            ("WITHOUT NODES")
            (num_nodes)
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
            ("NODES")
            self.nv = len(nodes)
            self.ne = self.nv - 1
            self.ndof = len(nodes) * 4 - 1
            self.rho = rho
            self.rod_radius = rod_radius
            self.youngM = youngs_modulus
            self.poisson_ratio = poisson_ratio
            self.mu = mu

            self.rod_length = sum(np.linalg.norm(nodes[i] - nodes[i - 1]) for i in range(1, self.nv))
        
        # Things required for setup function
        
        self.ncons = 0
        self.uncons = self.ndof
        
        ## Initialize constraint maps
        self.is_constrained = np.zeros(self.ndof, dtype=int)
        self.is_dof_joint = np.zeros(self.ndof, dtype=int)
        self.is_node_joint = np.zeros(self.nv, dtype=int)
        self.is_edge_joint = np.zeros(self.ne, dtype=int)

        # Initialize geometry
        self.setup(nodes)

        # print("RUNNING FOR SETUP")
        
        # Initialize state vectors
        # self.x0 = np.zeros(self.ndof)  # Previous timestep DOFs
        # self.x = np.zeros(self.ndof)   # Current timestep DOFs
        self.x_ls = np.zeros(self.ndof)  # Line search state
        # self.u0 = np.zeros(self.ndof)  # Previous timestep velocities
        # self.u = np.zeros(self.ndof)   # Current timestep velocities

    def setup(self, nodes: np.ndarray):
        """Setup basic rod geometry and allocate arrays."""
        
        self.x: np.ndarray = np.zeros(self.ndof)   # Current timestep DOFs

        for i in range(self.nv):
            self.x[4*i] = nodes[i][0]
            self.x[4*i+1] = nodes[i][1]
            self.x[4*i+2] = nodes[i][2]
            if i < self.nv - 1:
                self.x[4*i+3] = 0
        # ("X VALUES:", self.x)

        # print(self.x)

        self.x0 = np.copy(self.x)
        self.u = np.zeros(self.ndof)
        self.u0 = np.copy(self.u)

        # Unconstrained and constrained DOFs
        self.ncons = 0
        self.uncons = self.ndof
        self.is_constrained = np.zeros(self.ndof, dtype=int)
        self.is_dof_joint = np.zeros(self.ndof, dtype=int)
        self.is_node_joint = np.zeros(self.nv, dtype=int)
        self.is_edge_joint = np.zeros(self.ne, dtype=int)
        self.joint_ids = [(i, self.limb_idx) for i in range(self.nv)]

        # Setup the map from free dofs to all dof
        self.unconstrained_map = np.zeros(self.uncons, dtype=int) # maps xUncons to x
        self.full_to_uncons_map = np.zeros(self.ndof, dtype=int)

        self.setup_map()

        self.cross_sectional_area = math.pi * self.rod_radius * self.rod_radius

        # Initialize geometry arrays
        self.edge_len = np.zeros(self.ne)
        self.ref_len = np.zeros(self.ne)
        self.voronoi_len = np.zeros(self.nv)
    
        # Initialize frames
        self.d1 = np.zeros((self.ne, 3))
        self.d2 = np.zeros((self.ne, 3))
        self.d1_old = np.zeros((self.ne,3))
        self.d2_old = np.zeros((self.ne,3))
        self.m1 = np.zeros((self.ne, 3))
        self.m2 = np.zeros((self.ne, 3))
        self.tangent = np.zeros((self.ne, 3))
        
        # Initialize twists and curvatures
        self.ref_twist = np.zeros(self.ne)

        self.__set_reference_length()

        self.__set_mass()

        self.__compute_tangent()

        self.__compute_space_parallel()

        self.__compute_material_director()

        self.kb = np.zeros((self.nv, 3))
        self.twist_bar = np.zeros(self.ne)
        self.kappa = np.zeros((self.nv, 2))
        self.kappa_bar = np.zeros((self.nv, 2))

        self.ref_twist_old = np.zeros(self.ne)

        self.__get_ref_twist()

        self.__compute_twist_bar()

        self.__compute_kappa()

        self.kappa_bar = self.kappa.copy()

        self.__compute_edge_len()

        self.__compute_elastic_stiffness()

        self.d1_old = self.d1.copy()
        self.d2_old = self.d2.copy()
        self.ref_twist_old = self.ref_twist.copy()
        self.tangent_old = self.tangent.copy()
        
        # # Compute initial rod length
        # self.rod_length = 0
        # for i in range(self.ne):
        #     self.rod_length += np.linalg.norm(nodes[i+1] - nodes[i])

    def prepare_for_iteration(self):
        """Update discrete values and frames for the next timestep."""
        # print("PREPARE FOR ITERATIONS")
        self.__compute_tangent()
        self.__compute_time_parallel()
        self.__get_ref_twist()
        self.__compute_material_director()
        self.__compute_edge_len()
        self.__compute_kappa()

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
        # print("UPDATE NEWTON X")
        max_dx = 0.0
        for c in range(self.uncons):
            ind = self.unconstrained_map[c]
            self.x[ind] -= alpha * dx[offset + c]

            if (ind - 3) % 4 != 0:  # Non-theta degree of freedom
                curr_dx = abs(dx[offset + c])
                # print(dx[offset + c])
                if curr_dx > max_dx:
                    max_dx = curr_dx
        # print("MAX DX", max_dx)
        return max_dx

    def update_guess(self, weight:float, dt:float):
        """Update the guess for the next time step using a weighted combination of velocities and displacements."""
        # print("UNCONSTRAINED MAP", self.unconstrained_map)
        # print("BEFORE", self.x0)
        for c in range(self.uncons):
            ind = self.unconstrained_map[c]
            self.x[ind] = self.x0[ind] + weight * self.u[ind] * dt
        # print("X VALUE", self.x)
        # print("AFTER", self.x0)
            # print("self.x:" , ind , ":", self.x[ind])

    def enable_2d_sim(self):
        """Enable 2D simulation in x-z plane."""
        # print("ENABLE 2D SIm")
        for i in range(self.ne):
            self.is_constrained[4 * i + 1] = 1  # Constrain y-axis
            self.is_constrained[4 * i + 3] = 1  #

        self.is_constrained[4*self.ne + 1] = 1
    
    def get_vertex(self, k):
        """Return the position of the vertex at node k."""
        return self.x[4 * k: 4 * k + 3]

    def get_pre_vertex(self, k):
        """Return the initial position of the vertex at node k (before any update)."""
        return self.x0[4 * k: 4 * k + 3]

    def get_velocity(self, k):
        """Return the velocity of the vertex at node k."""
        return self.u[4 * k: 4 * k + 3]

    """
    setup_map function explained:
    
    Initialization (c = 0):

    A counter c is initialized to keep track of the position in the unconstrained_map array.
    Loop (for i in range(ndof)):

    A loop iterates over all the degrees of freedom (ndof). This loop runs from i = 0 to i = ndof - 1.
    Condition (if isConstrained[i] == 0 and isDOFJoint[i] != 1):

    Inside the loop, for each i, the condition checks:
    If isConstrained[i] == 0, meaning that the degree of freedom i is not constrained.
    If isDOFJoint[i] != 1, meaning that the degree of freedom i is not part of a joint.
    If both these conditions are satisfied, the degree of freedom i is considered "unconstrained."
    Mapping (unconstrained_map[c] = i and fullToUnconsMap[i] = c):

    The unconstrained_map[c] = i line assigns the index i of the unconstrained degree of freedom to the unconstrained_map at position c.
    The fullToUnconsMap[i] = c line creates a reverse mapping from the full degrees of freedom list (i) to the index c in the unconstrained_map.
    """

    def setup_map(self):
        # print("RUNNING SET UP")
        c = 0
        
        for i in range(self.ndof):
            if self.is_constrained[i] == 0 and self.is_dof_joint[i] != 1:
                self.unconstrained_map[c] = i
                self.full_to_uncons_map[i] = c
                c += 1
    
    def update_map(self):
        self.ncons = 0
        for i in range(self.ndof):
            if(self.is_constrained[i] > 0 or self.is_dof_joint[i] == 1):
                self.ncons+=1
            # self.ncons = np.sum((self.is_constrained > 0) | (self.is_dof_joint == 1))
        self.uncons = self.ndof - self.ncons

        self.unconstrained_map = np.zeros(self.uncons, dtype=int)
        self.full_to_uncons_map = np.zeros(self.ndof, dtype=int)
        self.setup_map()

    def add_joint(self, node_num: int, attach_to_joint: bool, joint_node: int, joint_limb: int ):
        if attach_to_joint:
            """
                Same as this code:
                isDOFJoint[4 * node_num] = 1;
                isDOFJoint[4 * node_num + 1] = 1;
                isDOFJoint[4 * node_num + 2] = 1;
            """
            self.is_dof_joint[4 * node_num: 4 * node_num + 3] = 1

            if node_num == 0:
                self.is_node_joint[0] = 1
                self.is_edge_joint[0] = 1
                self.joint_ids[0] = (joint_node, joint_limb)
            elif node_num == self.nv - 1:
                self.is_node_joint[self.nv - 1] = 1
                self.is_edge_joint[self.ne - 1] = 1
                self.joint_ids[self.nv - 1] = (joint_node, joint_limb)
            else:
                raise ValueError("Tried removing DOFs at the midpoint of an edge.")
        else:
            self.is_dof_joint[4 * node_num: 4 * node_num + 3] = 2

            if node_num == 0:
                self.is_node_joint[0] = 2
                self.is_edge_joint[0] = 2
            elif node_num == self.nv - 1:
                self.is_node_joint[self.nv - 1] = 2
                self.is_edge_joint[self.ne - 1] = 2
            else:
                self.is_node_joint[node_num] = 2
                self.is_edge_joint[node_num - 1] = 2
                self.is_edge_joint[node_num] = 2

    def set_vertex_boundary_condition(self,position: np.ndarray, k: int):
        self.is_constrained[4 * k: 4 * k + 3] = 1
        self.x[4 * k: 4 * k + 3] = position

    def set_theta_boundary_condition(self, desired_theta: float, k: int):
        self.is_constrained[4 * k + 3] = 1
        self.x[4 * k + 3] = desired_theta

    def free_vertex_boundary_condition(self, k: int):
        self.is_constrained[4 * k: 4 * k + 3] = 0

    def get_tangent(self, k: int):
        """Return the tangent vector at edge k."""
        return self.tangent[k]
    
    def get_theta(self, k: int):
        """Return the twist angle (theta) at node k."""
        return self.x[4 * k + 3]
    
    def __compute_elastic_stiffness(self):
        """Compute elastic stiffness parameters."""
        self.shearM = self.youngM / (2 * (1 + self.poisson_ratio))
        self.EA = self.youngM * self.cross_sectional_area
        self.EI = self.youngM * np.pi * self.rod_radius**4 / 4
        self.GJ = self.shearM * np.pi * self.rod_radius**4 / 2

    def __set_mass(self):
        # Assuming ndof, nv, ref_len, cross_sectional_area, rho, and rod_radius are already defined.
        self.mass_array = np.zeros(self.ndof)

        for i in range(self.nv):
            # Compute mass per unit length, dm, based on cross-sectional area and density.
            dm = 0.5 * self.cross_sectional_area * self.rho

            # Adjust dm based on the reference lengths for the first, last, and middle nodes.
            if i == 0:
                dm *= self.ref_len[i]
            elif i == self.nv - 1:
                dm *= self.ref_len[i - 1]
            else:
                dm *= (self.ref_len[i - 1] + self.ref_len[i])

            # Assign the computed mass to the first three components of the mass_array for each node.
            for k in range(3):
                self.mass_array[4 * i + k] = dm

            # For each node except the last, assign an additional mass component.
            if i < self.nv - 1:
                self.mass_array[4 * i + 3] = (self.rod_radius ** 2) * dm / 2.0

    def __set_reference_length(self):
        self.ref_len = np.zeros(self.ne)
        for i in range(self.ne):
            self.ref_len[i] = np.linalg.norm(self.x[4*(i+1):4*(i+1)+3] - self.x[4*i:4*i+3])

        self.voronoi_len = np.zeros(self.nv)
        for i in range(self.nv):
            if i == 0:
                self.voronoi_len[i] = 0.5 * self.ref_len[i]
            elif i == self.nv - 1:
                self.voronoi_len[i] = 0.5 * self.ref_len[i - 1]
            else:
                self.voronoi_len[i] = 0.5 * (self.ref_len[i - 1] + self.ref_len[i])

    def __compute_tangent(self):
        for i in range(self.ne):
            # Extract segments (3 elements) from 'x' to compute the tangent vector.
            self.tangent[i, :] = self.x[4 * (i + 1): 4 * (i + 1) + 3] - self.x[4 * i: 4 * i + 3]

            # Normalize the tangent vector.
            self.tangent[i, :] = self.tangent[i, :] / np.linalg.norm(self.tangent[i, :])
    
    def __compute_twist_bar(self):
        """Compute the twist deformation."""
        for i in range(1, self.ne):
            theta_i = self.x[4 * (i - 1) + 3]
            theta_f = self.x[4 * i + 3]
            self.twist_bar[i] = theta_f - theta_i + self.ref_twist[i]

    def __compute_time_parallel(self):
        for i in range(self.ne):
            t0 = self.tangent_old[i]
            t1 = self.tangent[i]
            d1_vector = np.zeros(3)
            d1_vector = self.__parallel_transport(self, self.d1_old[i], t0, t1, d1_vector)
            # print("D1 VECTOR", d1_vector)
            self.d1[i] = d1_vector
            self.d2[i] = np.cross(t1, d1_vector)

    def __compute_space_parallel(self):
        # This function is only called once
        t0 = self.tangent[0, :]  # First row of tangent vector
        t1 = np.array([0, 0, -1])  # Initialize t1 as a vector pointing down the z-axis
        d1_tmp = np.cross(t0, t1)  # Compute the cross product of t0 and t1

        # Check if the magnitude (norm) of d1_tmp is close to zero
        if np.linalg.norm(d1_tmp) < 1.0e-6:
            # If cross product was too small, choose another orthogonal vector t1
            t1 = np.array([0, 1, 0])  # A vector along the y-axis
            d1_tmp = np.cross(t0, t1)  # Compute the cross product again

        self.d1[0, :] = d1_tmp  # Store the result in the first row of d1
        self.d2[0, :] = np.cross(t0, d1_tmp)  # Store the cross product of t0 and d1_tmp in d2

        # print("ROW d1", d1_tmp)
        # print("ROW d1", self.d2)

        # Loop over all elements and compute space-parallel transport
        for i in range(self.ne - 1):
            a = self.d1[i, :]
            b = self.tangent[i, :]
            c = self.tangent[i + 1, :]
            d = np.zeros(3)
            
            # Call parallelTransport (assuming you have a Python function for it)
            d = self.__parallel_transport(self, a, b, c, d)  # Perform parallel transport
            self.d1[i + 1, :] = d  # Store result in the next row of d1
            self.d2[i + 1, :] = np.cross(c, d)  # Store the cross product in d2

    def __compute_material_director(self):
        global cs,ss
        global angle

        for i in np.arange(self.ne):
            angle = self.x[4*i+3] # Extract the angle from x
            cs = math.cos(angle) # Compute the cosine of the angle
            ss = math.sin(angle) # Compute the sine of the angle

            # Update m1 and m2 based on the cosine and sine values
            # print("D1", i , self.d1)
            self.m1[i, :] = cs * self.d1[i, :] + ss * self.d2[i, :]
            self.m2[i, :] = -ss * self.d1[i, :] + cs * self.d2[i, :]

        # print("M1: ", self.m1[i, :])

    def __compute_edge_len(self):
        """Compute the length of each edge."""
        for i in range(self.ne):
            self.edge_len[i] = np.linalg.norm(self.x[4 * (i + 1): 4 * (i + 1) + 3] - self.x[4 * i: 4 * i + 3])

    def __get_ref_twist(self):
        # nv = len(refTwist)
        # self.ne = self.nv - 1
        for i in np.arange(1,self.ne):
            u0 = self.d1[i-1]
            u1 = self.d1[i]
            t0 = self.tangent[i-1]
            t =  self.tangent[i]
            ut = np.zeros(3)
            ut = self.__parallel_transport(self, u0,t0,t, ut)
            ut = self.__rotate_axis_angle(self, ut,t,self.ref_twist_old[i])
            sgnAngle = self.__signed_angle(self, ut,u1,t)
            self.ref_twist[i] = self.ref_twist_old[i] + sgnAngle

        # return self.ref_twist

    def __compute_kappa(self):
        # print("KAPPA COMPUTING")
        # First loop: Compute kb using the tangent vectors
        for i in range(1, self.ne):
            t0 = self.tangent[i - 1, :]  # Get the (i-1)th row of the tangent array
            t1 = self.tangent[i, :]      # Get the ith row of the tangent array
            self.kb[i, :] = 2.0 * np.cross(t0, t1) / (1.0 + np.dot(t0, t1))

        # print("KB Row:", i , ":", self.kb[i, :])

        # Second loop: Compute kappa using m1, m2, and kb
        for i in range(1, self.ne):
            m1e = self.m1[i - 1, :]  # Get the (i-1)th row of m1
            m2e = self.m2[i - 1, :]  # Get the (i-1)th row of m2
            m1f = self.m1[i, :]      # Get the ith row of m1
            m2f = self.m2[i, :]      # Get the ith row of m2
            # print("m1e", m1e)

            # Calculate the values for kappa
            self.kappa[i, 0] = 0.5 * np.dot(self.kb[i, :], (m2e + m2f))  # First component of kappa
            self.kappa[i, 1] = -0.5 * np.dot(self.kb[i, :], (m1e + m1f))  # Second component of kappa

            # print("KAPPA1", self.kappa[i,0])
            # print("KAPPA2", self.kappa[i,1])

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
        # Assuming t1, t2, and d1_1 are numpy arrays representing 3D vectors.
        
        # Cross product of t1 and t2
        b = np.cross(t1, t2)

        # Check if the norm of b is 0
        if np.linalg.norm(b) == 0:
            d1_2 = d1_1
        else:
            # Normalize b and make it orthogonal to t1 and t2
            b = b / np.linalg.norm(b)
            b = b - np.dot(b, t1) * t1
            b = b / np.linalg.norm(b)
            b = b - np.dot(b, t2) * t2
            b = b / np.linalg.norm(b)

            # Compute n1 and n2 as cross products
            n1 = np.cross(t1, b)
            n2 = np.cross(t2, b)

            # Compute d1_2 based on dot products and projections
            d1_2 = np.dot(d1_1,t1) * t2 + np.dot(d1_1,n1) * n2 + np.dot(d1_1,b) * b
        
        return d1_2

    @staticmethod
    def __rotate_axis_angle(self, v: np.ndarray, z: np.ndarray, theta: float):
        """
        Rotate vector v around axis z by angle theta.
        
        Args:
            v: Vector to rotate
            z: Rotation axis
            theta: Rotation angle
        """
        if theta == 0:
            return v
        else: 
            c = np.cos(theta)
            s = np.sin(theta)
            v[:] = v * c + np.cross(z, v) * s + z * np.dot(z, v) * (1 - c)
            return v[:]

    @staticmethod
    def __signed_angle(self, u: np.ndarray,v: np.ndarray, n: np.ndarray):
        w = np.cross(u,v)
        angle = np.arctan2( np.linalg.norm(w), np.dot(u,v) )
        if (np.dot(n,w) < 0):
            angle = -angle

        return angle