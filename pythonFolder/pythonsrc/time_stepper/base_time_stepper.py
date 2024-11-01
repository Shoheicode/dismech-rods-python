import numpy as np
from abc import ABC, abstractmethod

from pythonsrc.rod_mechanics.elastic_rod import ElasticRod
from pythonsrc.globalDefinitions import SimParams
from pythonsrc.rod_mechanics.force_container import ForceContainer
from pythonsrc.rod_mechanics.soft_robots import SoftRobots

class BaseTimeStepper(ABC):
    def __init__(self, soft_robots: SoftRobots, forces: ForceContainer, sim_params: SimParams):
        # Constructor for baseTimeStepper class
        self.forces = forces            # shared_ptr<forceContainer> equivalent
        self.sim_params = sim_params    # simParams equivalent

        self.freeDOF = 0

        # References for shared objects
        self.limbs = soft_robots.limbs        # Vector of shared_ptr<elasticRod>
        self.joints = soft_robots.joints      # Vector of shared_ptr<elasticJoint>
        self.controllers = soft_robots.controllers  # Vector of shared_ptr<baseController>

        self.offsets = []                 # Equivalent to vector<int> offsets

        for limb in self.limbs:
            self.offsets.append(self.freeDOF)
            self.freeDOF += limb.uncons

        # self.force = np.zeros(self.freeDOF)
        
        # Initialize variables
        self.dx = np.zeros(self.freeDOF)     # Equivalent to double* dx
        self.force = np.zeros(self.freeDOF)  # Equivalent to double* force
        self.Force = np.zeros(self.freeDOF)  # Map<VectorXd> Force
        self.DX = np.zeros(self.freeDOF)     # Map<VectorXd> DX
        self.Jacobian = np.zeros((self.freeDOF, self.freeDOF))  # MatrixXd Jacobian
        
        self.freeDOF = self.freeDOF  # Integer for free degrees of freedom
        self.iter = 0                     # Iteration count

        self.mappedInd = None
        self.mappedInd1 = None
        self.mappedInd2 = None
        self.offset = None
        self.dt = sim_params.dt
        self.alpha = 1.0
    
    # Add force to the system
    def add_force(self, ind, p, limb_idx):
        """
        Add a force to a specific degree of freedom for a given limb.
        
        Parameters:
            ind (int): Index of the degree of freedom.
            p (float): Force value to add.
            limb_idx (int): Index of the limb.
        """
        # print("HIHIHI")
        limb:ElasticRod = self.limbs[limb_idx]
        offset = self.offsets[limb_idx]

        if limb.is_constrained[ind] == 0:  # Free DOF
            mapped_ind = limb.full_to_uncons_map[ind]
            self.force[mapped_ind + offset] += p
    
    # Abstract methods (pure virtual in C++)
    def init_stepper(self):
        self.forces.setup_force_stepper_access(self)
    
    @abstractmethod
    def prep_system_for_iteration(self):
        pass

    def prep_system_for_iteration(self):
        # print("HIHIH")
        for self.joint in self.joints:
            self.joint.prep_limbs()
        for self.limb in self.limbs:
            self.limb.prepare_for_iteration()
        for self.joint in self.joints:
            self.joint.prepare_for_iteration()
    
    @abstractmethod
    def set_zero(self):
        pass

    def set_zero(self):
        """Set the Force array to zero."""
        self.force.fill(0)
    
    @abstractmethod
    def update(self):
        pass

    def update(self):
        """Recalculate degrees of freedom, offsets, and reset force and dx arrays."""
        self.freeDOF = 0
        self.offsets = []
        
        for limb in self.limbs:
            self.offsets.append(self.freeDOF)
            self.freeDOF += limb.uncons
        
        # Reset force and dx arrays
        self.force = np.zeros(self.freeDOF)
        self.dx = np.zeros(self.freeDOF)

        self.Force = self.force
    
    @abstractmethod
    def integrator(self):
        pass
    
    @abstractmethod
    def add_jacobian(self, ind1, ind2, p, limb_idx):
        pass
    
    def add_jacobian_2(self, ind1, ind2, p, limb_idx1, limb_idx2):
        pass
    
    @abstractmethod
    def update_system_for_next_time_step(self):
        pass

    def update_system_for_next_time_step(self):
        pass
    
    @abstractmethod
    def step_forward_in_time(self):
        pass