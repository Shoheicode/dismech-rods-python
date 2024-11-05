import numpy as np
from pythonsrc.solvers.pardiso_solver import PardisoSolver
from pythonsrc.solvers.base_solver import BaseSolver
from pythonsrc.rod_mechanics.elastic_rod import ElasticRod
from pythonsrc.globalDefinitions import IntegratorMethod, SimParams
from pythonsrc.time_stepper.base_time_stepper import BaseTimeStepper
from typing import List, Tuple, Optional

from pythonsrc.solvers.solver_types import SolverType

class ImplicitTimeStepper(BaseTimeStepper):
    def __init__(self, soft_robots, forces, sim_params: SimParams, solver_type: str):
        super().__init__(soft_robots, forces, sim_params)
        self.solver_type = solver_type

        # Utility variables for the PARDISO solver
        self.ia = []  # Assuming this is a list of integers (MKL_INT* equivalent)
        self.non_zero_elements = []  # List of tuples to store non-zero element indices
        self.dgbsv_jacobian = None  # Placeholder for dgbsv Jacobian array
        self.kl = 0
        self.ku = 0
        self.num_rows = 0

        # Protected attributes
        self.ftol = sim_params.ftol
        self.dtol = sim_params.dtol
        self.max_iter = sim_params.max_iters["num_iters"]
        self.terminate_at_max = sim_params.max_iters["terminate_at_max"]
        self.line_search = sim_params.line_search
        self.orig_dt = sim_params.dt
        self.adaptive_time_stepping_threshold = sim_params.adaptive_time_stepping
        self.adaptive_time_stepping = sim_params.adaptive_time_stepping != 0
        self.solver_type = solver_type

        self.Jacobian = np.zeros((self.freeDOF, self.freeDOF))
        self.dgbsv_jacobian = None
        self.ia = None

        # Private attributes
        self.solver:BaseSolver = None  # Unique pointer equivalent to `baseSolver`
        self.dgbsv_jacobian_len = 0

    def add_jacobian(self, ind1, ind2, p, limb_idx1, limb_idx2=None):
        # print("ADD JACOBIAN")
        if limb_idx2 is None:
            # print("LIMB IS NONE")
            limb_idx2 = limb_idx1

        limb1: ElasticRod = self.limbs[limb_idx1]
        limb2: ElasticRod = self.limbs[limb_idx2]
        self.mapped_ind1 = limb1.full_to_uncons_map[ind1]
        self.mapped_ind2 = limb2.full_to_uncons_map[ind2]
        offset1 = self.offsets[limb_idx1]
        offset2 = self.offsets[limb_idx2]
        jac_ind1 = self.mapped_ind2 + offset2
        jac_ind2 = self.mapped_ind1 + offset1

        if limb1.is_constrained[ind1] == 0 and limb2.is_constrained[ind2] == 0:
            if self.solver_type == "PARDISO_SOLVER":
                if self.Jacobian[jac_ind1, jac_ind2] == 0 and p != 0:
                    # print("JACOBIAN self time")
                    self.ia[jac_ind1 + 1] += 1
                    self.non_zero_elements.append((jac_ind1, jac_ind2))
                elif self.Jacobian[jac_ind1, jac_ind2] != 0 and self.Jacobian[jac_ind1, jac_ind2] + p == 0:
                    print("JACOBIAN not equal time")
                    self.ia[jac_ind1 + 1] -= 1
                    self.non_zero_elements.remove((jac_ind1, jac_ind2))
                self.Jacobian[jac_ind1, jac_ind2] += p
            elif self.solver_type == "DGBSV_SOLVER":
                dgbsv_row = self.kl + self.ku + jac_ind1 - jac_ind2
                dgbsv_col = jac_ind2
                dgbsv_offset = dgbsv_row + dgbsv_col * self.num_rows
                self.dgbsv_jacobian[dgbsv_offset] += p
                self.Jacobian[jac_ind1, jac_ind2] += p

    def set_zero(self):
        # Reset or zero out any matrices or structures as necessary.
        super().set_zero()
        if self.solver_type == "PARDISO_SOLVER":
            self.non_zero_elements.clear()
            self.ia.fill(0)
            self.ia[0] = 1
        elif self.solver_type == "DGBSV_SOLVER":
            self.dgbsv_jacobian.fill(0)
        self.Jacobian.fill(0)

    def update(self):
        # Update the system state; implementation needed based on your requirements.
        super().update()
        # print("UPDATE IS RUNNING")
        if self.solver_type == "PARDISO_SOLVER":
            # print("I AM RUNNING PARDISO")
            self.ia = np.zeros(self.freeDOF + 1, dtype=int)
            self.ia[0] = 1
        elif self.solver_type == "DGBSV_SOLVER":
            local_solver = self.solver
            self.dgbsv_jacobian_len = local_solver.NUMROWS * self.freeDOF
            self.dgbsv_jacobian = np.zeros(self.dgbsv_jacobian_len)
        self.Jacobian.fill(0)

    def integrator(self):
        # Integrate using this timestepper
        # print("This is my solver:", self.solver_type)
        self.solver.integrator()

    def init_stepper(self):
        # print("IMPLICIT TIME STOPPER")
        # Initialize the stepper; set any parameters or configurations.
        super().init_stepper()
        if self.solver_type == "PARDISO_SOLVER":
            # print("PAR SOLVER")
            self.solver = PardisoSolver(self)# SolverType.PARDISO_SOLVER
            self.ia = np.zeros(self.freeDOF + 1, dtype=int)
            self.ia[0] = 1
        elif self.solver_type == "DGBSV_SOLVER":
            self.solver = SolverType.DGBSV_SOLVER
            local_solver = self.solver
            self.dgbsv_jacobian_len = local_solver.NUMROWS * self.freeDOF
            self.kl = local_solver.kl
            self.ku = local_solver.ku
            self.num_rows = local_solver.NUMROWS
            self.dgbsv_jacobian = np.zeros(self.dgbsv_jacobian_len)

    def prep_system_for_iteration(self):
        # Prepare the system for the next iteration.
        # print("PREP")
        super().prep_system_for_iteration()
        self.set_zero()

    def newton_method(self, dt):
        # Pure virtual method in C++; define it as abstract in subclasses.
        raise NotImplementedError("This method should be overridden in a subclass")

    def line_search_func(self, dt):
        # Pure virtual method in C++; define it as abstract in subclasses.
        raise NotImplementedError("This method should be overridden in a subclass")

    def shared_from_this(self):
        # In Python, `shared_from_this` is not typically needed as Python manages references automatically.
        return self
