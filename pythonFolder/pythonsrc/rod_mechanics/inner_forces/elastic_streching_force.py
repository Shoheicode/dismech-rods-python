from typing import List
from pythonsrc.rod_mechanics.base_force import BaseForce
from pythonsrc.rod_mechanics.elastic_rod import ElasticRod
from pythonsrc.rod_mechanics.soft_robots import SoftRobots
import numpy as np

class ElasticStretchingForce(BaseForce):
    
    def __init__(self, soft_robots: SoftRobots):
        # Initialize class attributes related to forces and Jacobians
        self.len : float = 0
        self.ref_length :float = 0
        self.epsX: float = 0  # Strain in the material
        self.u = np.zeros(3)  # Displacement vector
        self.dxx = np.zeros(3)  # Vector for calculating displacement
        self.f = np.zeros(3)  # Force vector
        self.Id3 = np.eye(3)  # 3x3 identity matrix
        self.M0 = np.zeros((3, 3))  # Jacobian matrix component for elasticity
        self.v = np.zeros((1, 3))  # Transposed displacement vector
        self.JSS = np.zeros((7,7))  # 7x7 Jacobian matrix for stiffness

        # Call parent class constructor
        super().__init__(soft_robots)

    # Function to compute the elastic stretching force for each limb and joint
    def compute_force(self, dt):
        limb_idx = 0  # Keep track of limb index
        for limb in self.soft_robots.limbs:
            # print("LIMBS COMPUTE FORCE STRETCHING", limb)
            for i in range(limb.ne):  # Iterate over each edge in the limb
                if limb.is_edge_joint[i]:
                    continue  # Skip if the edge is part of a joint

                # Calculate strain (epsX) for the edge based on current and reference length
                self.epsX = limb.edge_len[i] / limb.ref_len[i] - 1.0

                # Calculate the force vector 'f' using EA (Young's modulus * cross-sectional area)
                self.f = limb.EA * limb.tangent[i, :] * self.epsX  # NumPy row access

                # Apply forces to the start and end nodes of the edge
                # print("STEPPER", self.stepper)
                for k in range(3):
                    ind = 4 * i + k  # Index for the first node
                    self.stepper.add_force(ind, -self.f[k], limb_idx)  # Subtract elastic force

                    ind = 4 * (i + 1) + k  # Index for the second node
                    self.stepper.add_force(ind, self.f[k], limb_idx)  # Add elastic force

            limb_idx += 1  # Move to the next limb
        
        # Handle the joints in the soft robot
        for joint in self.soft_robots.joints:
            print("JOINTS")
            for i in range(joint.ne):  # Iterate over each edge in the joint
                sgn = 1 if joint.bending_twist_signs[i] == 1 else -1  # Determine sign of bending/twist
                n1 = joint.connected_nodes[i][0]  # First connected node
                limb_idx = joint.connected_nodes[i][1]  # Limb index where node is connected
                curr_limb = self.soft_robots.limbs[limb_idx]  # Get current limb

                # Calculate strain (epsX) for the joint based on current and reference length
                self.epsX = joint.edge_len[i] / joint.ref_len[i] - 1.0

                # Calculate force 'f' for the joint
                self.f = curr_limb.EA * joint.tangents[i, :] * sgn * self.epsX  # NumPy row access

                # Apply forces to the nodes of the joint
                for k in range(3):
                    ind = 4 * n1 + k  # Index for the connected node
                    self.stepper.add_force(ind, -self.f[k], limb_idx)  # Subtract force from node

                    ind = 4 * joint.joint_node + k  # Index for the joint node itself
                    self.stepper.add_force(ind, self.f[k], joint.joint_limb)  # Add force to the joint limb

    # Function to compute both the force and Jacobian (stiffness matrix) for each limb and joint
    def compute_force_and_jacobian(self, dt):
        self.compute_force(dt)  # First, compute the forces

        limb_idx = 0  # Keep track of limb index
        for limb in self.soft_robots.limbs:
            # print("LIMB", limb.EA)
            # print("LIMB NE", limb.ne)
            for i in range(limb.ne):
                if limb.is_edge_joint[i]:
                    continue  # Skip if the edge is part of a joint


                # print("DIFFERENT TIME")
                # Calculate current and reference lengths of the edge
                self.len = limb.edge_len[i]
                self.ref_length = limb.ref_len[i]

                # Calculate the difference vector (dxx) between nodes
                self.dxx = np.zeros(3)
                self.dxx[0] = limb.x[4*i+4] - limb.x[4*i+0]
                self.dxx[1] = limb.x[4*i+5] - limb.x[4*i+1]
                self.dxx[2] = limb.x[4*i+6] - limb.x[4*i+2]

                # if(i < 50):
                #     print("dxx 0 of ",i, " ",self.dxx[0])
                #     print("dxx 1 of ",i, " ",self.dxx[1])
                #     print("dxx 2 of ",i, " ",self.dxx[2])
                
                # Define u and v (displacement vectors)
                self.u = self.dxx
                self.v = self.u.reshape(-1, 1)  # Reshape to a column vector

                # Compute M0 matrix for the edge, based on elasticity and displacement
                self.M0 = limb.EA * ((1 / self.ref_length - 1 / self.len) * self.Id3 + 
                                     (1 / self.len) * np.outer(self.u, self.u) / (self.u.dot(self.u)))
                
                # print("M0 Value" , i , ": ", self.M0)
                
                # print("M0000 VALUE",i, self.M0)
                # print("")

                # Update the blocks of the 7x7 Jacobian matrix
                self.JSS[0:3, 0:3] = -self.M0
                self.JSS[4:7, 4:7] = -self.M0
                self.JSS[4:7, 0:3] = self.M0
                self.JSS[0:3, 4:7] = self.M0

                # print("self.JSS[0:3, 0:3]", i , ": ", self.JSS[0:3, 0:3])
                # print("")

                # Add the Jacobian components to the stepper for both nodes of the edge
                for j in range(7):
                    for k in range(7):
                        ind1 = 4*i + j
                        ind2 = 4*i + k
                        # print("IND1: ", ind1, "IND2: ", ind2)
                        # print("JSS VALUE, k: ", k ,"j :",  j, -self.JSS[k, j])
                        self.stepper.add_jacobian(ind1, ind2, -self.JSS[k, j], limb_idx)

            limb_idx += 1  # Move to the next limb

        # Process the joints in the soft robot for the Jacobian
        for joint in self.soft_robots.joints:
            print("JOINNTTTTs")
            for i in range(joint.ne):
                n1 = joint.connected_nodes[i][0]  # First connected node of the joint
                limb_idx = joint.connected_nodes[i][1]  # Limb index
                curr_limb = self.soft_robots.limbs[limb_idx]

                # Calculate current and reference lengths for the joint
                self.len = joint.edge_len[i]
                refLength = joint.ref_len[i]

                # Calculate the difference vector (dxx) between joint node and limb node
                self.dxx = np.zeros(3)
                self.dxx[0] = joint.x[0] - curr_limb.x[4*n1]
                self.dxx[1] = joint.x[1] - curr_limb.x[4*n1+1]
                self.dxx[2] = joint.x[2] - curr_limb.x[4*n1+2]

                # Define u and v (displacement vectors)
                self.u = self.dxx
                self.v = self.u.reshape(-1, 1)  # Reshape to a column vector

                # Compute M0 matrix for the joint
                self.M0 = curr_limb.EA * ((1 / refLength - 1 / self.len) * self.Id3 + 
                                          (1 / self.len) * np.outer(self.u, self.u) / (self.u.dot(self.u)))

                # Update the Jacobian blocks for the joint
                self.JSS[0:3, 0:3] = -self.M0
                self.JSS[4:7, 4:7] = -self.M0
                self.JSS[0:3, 4:7] = self.M0
                self.JSS[4:7, 0:3] = self.M0

                # Apply the Jacobian to the stepper using n1 and joint node
                l1 = limb_idx
                l2 = joint.joint_limb
                n2 = joint.joint_node

                # Add Jacobian terms for the node-node, node-joint, joint-node, and joint-joint interactions
                for j in range(3):
                    for k in range(3):
                        self.stepper.add_jacobian(4*n1 + j, 4*n1 + k, -self.JSS[k, j], l1)
                        self.stepper.add_jacobian(4*n1 + j, 4*n2 + k, -self.JSS[k + 4, j], l1, l2)
                        self.stepper.add_jacobian(4*n2 + j, 4*n1 + k, -self.JSS[k, j + 4], l2, l1)
                        self.stepper.add_jacobian(4*n2 + j, 4*n2 + k, -self.JSS[k + 4, j + 4], l2)