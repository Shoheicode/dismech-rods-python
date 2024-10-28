from typing import List
import numpy as np
from pythonsrc.rod_mechanics.elastic_rod import ElasticRod
from pythonsrc.rod_mechanics.elasticjoint import ElasticJoint


class SoftRobots():
    def __init__(self):
        self.limbs :List[ElasticRod] = [] # List of elastic rod (limbs)
        self.joints :List[ElasticJoint] = [] # List of elastic joints
        self.controllers = []
        self.num_limbs = 0  # Counter to track the number of limbs

    # Method to add a limb using start and end points with other parameters
    def add_limb(self, start: np.ndarray, end: np.ndarray, num_nodes: int, rho: float, rod_radius: float, 
                 youngs_modulus: float, poisson_ratio: float, mu: float = 0.0):
        limb = ElasticRod(self.num_limbs, start, end, num_nodes, None, rho, rod_radius, youngs_modulus, poisson_ratio, mu)
        self.limbs.append(limb)  # Add new limb to the list
        self.num_limbs += 1  # Increment limb counter

    # Method to add a limb using a list of nodes
    def add_limb_with_nodes(self, nodes, rho, rod_radius, youngs_modulus, poisson_ratio, mu = 0.0):
        limb = ElasticRod(self.num_limbs,None, None, len(nodes), nodes, rho, rod_radius, youngs_modulus, poisson_ratio, mu)
        self.limbs.append(limb)  # Add limb to the list
        self.num_limbs += 1  # Increment limb counter

    # Method to create a joint between limbs
    def create_joint(self, limb_idx, m_node_idx):
        # Default to the last node if m_node_idx is -1
        node_idx = m_node_idx if m_node_idx != -1 else self.limbs[limb_idx].nv - 1
        joint = ElasticJoint(node_idx, limb_idx, self.limbs)
        self.joints.append(joint)  # Add joint to the list

    # Method to add a limb to an existing joint
    def add_to_joint(self,joint_idx, limb_idx, m_node_idx):
        # Default to the last node if m_node_idx is -1
        node_idx = m_node_idx if m_node_idx != -1 else self.limbs[limb_idx].nv - 1
        self.joints[joint_idx].add_to_joint(node_idx, limb_idx)  # Add limb to the existing joint

    # Method to lock an edge of a limb (by setting boundary conditions)
    def lockEdge(self, limb_idx, edge_idx):
        limb : ElasticRod = self.limbs[limb_idx]
        # Set boundary conditions on the two vertices of the edge
        limb.set_vertex_boundary_condition(limb.get_vertex(edge_idx), edge_idx)
        limb.set_vertex_boundary_condition(limb.get_vertex(edge_idx + 1), edge_idx + 1)
        # Set boundary condition for twist angle (theta)
        limb.set_theta_boundary_condition(0.0, edge_idx)

    # Method to apply initial velocities to the limb's nodes
    def applyInitialVelocities(self, limb_idx, velocities):
        limb : ElasticRod = self.limbs[limb_idx]
        if limb.nv != len(velocities):
            raise ValueError(f"The number of nodes ({limb.nv}) and velocities ({len(velocities)}) given did not match!")
        for i in range(limb.nv):
            # Set initial velocities for the limb's nodes
            limb.u[4*i:4*i+3] = limb.u0[4*i:4*i+3] = velocities[i]

    # Method to set up joints and their configurations
    def setup(self):
        for joint in self.joints:
            joint.setup()  # Call setup for each joint

    # Method to add a controller to the robot
    def add_controller(self,controller):
        self.controllers.append(controller)  # Add controller to the list


