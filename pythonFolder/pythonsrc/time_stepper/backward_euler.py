import numpy as np

from pythonsrc.time_stepper.implicit_time_stepper import ImplicitTimeStepper
from pythonsrc.globalDefinitions import SimParams
from pythonsrc.rod_mechanics.force_container import ForceContainer
from pythonsrc.rod_mechanics.soft_robots import SoftRobots
from pythonsrc.solvers.solver_types import SolverType

class BackwardEuler(ImplicitTimeStepper):
    def __init__(self, soft_robots: SoftRobots, forces: ForceContainer, sim_params: SimParams, solver_type: SolverType):
        super().__init__(soft_robots, forces, sim_params, solver_type)

    def __del__(self):
        # Destructor equivalent, if needed in Python, though typically handled by Python's garbage collection.
        pass

    def update_system_for_next_time_step(self):
        # Override with the implementation of system update logic.
        pass

    def newton_method(self, dt):
        # Override with Newton's method implementation.
        normf = 0
        normf0 = 0
        solved = False
        self.iter = 0

        while not solved:
            self.prep_system_for_iteration()
            self.forces.compute_forces_and_jacobian(dt)

            # Compute norm of the force equations
            normf = np.linalg.norm(self.Force)

            if self.iter == 0:
                normf0 = normf

            # Force tolerance check
            if normf <= normf0 * self.ftol:
                solved = True
                self.iter += 1
                continue

            # Adaptive time stepping if enabled
            if self.adaptive_time_stepping and self.iter != 0 and self.iter % self.adaptive_time_stepping_threshold == 0:
                dt *= 0.5
                for limb in self.limbs:
                    limb.update_guess(0.01, dt)
                self.iter += 1
                continue

            # Solve equations of motion
            self.integrator()
            if self.line_search_enabled:
                self.line_search(dt)

            # Newton update
            max_dx = 0
            for idx, limb in enumerate(self.limbs):
                curr_dx = limb.update_newton_x(self.dx, self.offsets[idx], self.alpha)
                max_dx = max(max_dx, curr_dx)

            # Dynamics tolerance check
            if max_dx / dt < self.dtol:
                solved = True
                self.iter += 1
                continue

            self.iter += 1

            # Exit if unable to converge
            if self.iter > self.max_iter:
                if self.terminate_at_max:
                    print(f"No convergence after {self.max_iter} iterations")
                    exit(1)
                else:
                    solved = True

        return dt

    def line_search(self, dt):
        # Override with line search implementation.
        pass

    def step_forward_in_time(self):
        # Override with time-stepping logic.
        return 0.0  # Replace with the actual return value from the method's computation
