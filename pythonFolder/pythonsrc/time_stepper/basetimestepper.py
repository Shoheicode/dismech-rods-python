import numpy as np
from abc import ABC, abstractmethod

class BaseTimeStepper(ABC):
    def __init__(self, soft_robots, forces, sim_params):
        # Constructor for baseTimeStepper class
        self.soft_robots = soft_robots  # shared_ptr<softRobots> equivalent
        self.forces = forces            # shared_ptr<forceContainer> equivalent
        self.sim_params = sim_params    # simParams equivalent
        
        # Initialize variables
        self.dx = np.zeros(sim_params.freeDOF)     # Equivalent to double* dx
        self.force = np.zeros(sim_params.freeDOF)  # Equivalent to double* force
        self.Force = np.zeros(sim_params.freeDOF)  # Map<VectorXd> Force
        self.DX = np.zeros(sim_params.freeDOF)     # Map<VectorXd> DX
        self.Jacobian = np.zeros((sim_params.freeDOF, sim_params.freeDOF))  # MatrixXd Jacobian
        
        self.freeDOF = sim_params.freeDOF  # Integer for free degrees of freedom
        self.offsets = []                 # Equivalent to vector<int> offsets
        self.iter = 0                     # Iteration count
        
        # References for shared objects
        self.limbs = soft_robots.limbs        # Vector of shared_ptr<elasticRod>
        self.joints = soft_robots.joints      # Vector of shared_ptr<elasticJoint>
        self.controllers = soft_robots.controllers  # Vector of shared_ptr<baseController>

        self.mappedInd = None
        self.mappedInd1 = None
        self.mappedInd2 = None
        self.offset = None
        self.dt = sim_params.dt
        self.alpha = 1.0
    
    # Add force to the system
    def add_force(self, ind, p, limb_idx):
        self.force[ind] += p
    
    # Abstract methods (pure virtual in C++)
    @abstractmethod
    def init_stepper(self):
        pass
    
    @abstractmethod
    def prep_system_for_iteration(self):
        pass
    
    @abstractmethod
    def set_zero(self):
        pass
    
    @abstractmethod
    def update(self):
        pass
    
    @abstractmethod
    def integrator(self):
        pass
    
    @abstractmethod
    def add_jacobian(self, ind1, ind2, p, limb_idx):
        pass
    
    @abstractmethod
    def add_jacobian_2(self, ind1, ind2, p, limb_idx1, limb_idx2):
        pass
    
    @abstractmethod
    def update_system_for_next_time_step(self):
        pass
    
    @abstractmethod
    def step_forward_in_time(self):
        pass