import numpy as np

from pythonsrc.time_stepper.implicit_time_stepper import ImplicitTimeStepper
from pythonsrc.globalDefinitions import SimParams
from pythonsrc.rod_mechanics.force_container import ForceContainer
from pythonsrc.rod_mechanics.soft_robots import SoftRobots
from pythonsrc.solvers.solver_types import SolverType

class BackwardEuler(ImplicitTimeStepper):
    def __init__(self, soft_robots: SoftRobots, forces: ForceContainer, sim_params: SimParams, solver_type: str):
        super().__init__(soft_robots, forces, sim_params, solver_type)

    def __del__(self):
        # Destructor equivalent, if needed in Python, though typically handled by Python's garbage collection.
        pass

    def newton_method(self, dt):
        # Override with Newton's method implementation.
        normf = 0
        normf0 = 0
        solved = False
        self.iter = 0

        # print("NEWTON METHOD RUNNING")

        while not solved:
            self.prep_system_for_iteration()
            self.forces.compute_forces_and_jacobian(dt)
            print(dt)

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
            if self.line_search:
                self.line_search_func(dt)

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

        # print("SOLVED")

        return dt

    def line_search_func(self, dt):
        # Override with line search implementation.
        # Store current positions
        for limb in self.limbs:
            limb.x_ls = limb.x.copy()
        for joint in self.joints:
            joint.x_ls = joint.x.copy()

        amax, amin = 2, 1e-3
        al, au = 0, 1
        a = 1

        # Compute the initial slope
        q0 = 0.5 * np.power(np.linalg.norm(self.Force), 2)
        dq0 = -(self.Force.T @ self.Jacobian @ self.DX)[0]

        success = False
        m2, m1 = 0.9, 0.1

        while not success:
            for joint in self.joints:
                joint.x = joint.x_ls.copy()
            for idx, limb in enumerate(self.limbs):
                limb.x = limb.x_ls.copy()
                limb.update_newton_x(self.dx, self.offsets[idx], self.alpha)

            self.prep_system_for_iteration()
            self.forces.compute_force(dt)

            q = 0.5 * np.power(np.linalg.norm(self.Force), 2)
            slope = (q - q0) / a

            if m2 * dq0 <= slope <= m1 * dq0:
                success = True
            else:
                if slope < m2 * dq0:
                    al = a
                else:
                    au = a

                a = 0.5 * (al + au) if au < amax else 10 * a

            if a > amax or a < amin:
                break

        # Reset positions with optimal alpha
        self.alpha = a
        for limb in self.limbs:
            limb.x = limb.x_ls.copy()
        for joint in self.joints:
            joint.x = joint.x_ls.copy()

    def step_forward_in_time(self):
        # Override with time-stepping logic.
        self.dt = self.orig_dt

        # Initial guess using the last solution
        for limb in self.limbs:
            limb.update_guess(0.01, self.dt)

        # Perform collision detection if contact is enabled
        if self.forces.cf:
            self.forces.cf.broad_phase_collision_detection()

        # Run Newton's method to find the next time step
        self.dt = self.newton_method(self.dt)

        # Update limb positions and velocities
        for limb in self.limbs:
            limb.u = (limb.x - limb.x0) / self.dt
            limb.x0 = limb.x

        self.update_system_for_next_time_step()
        return self.dt
    
    def update_system_for_next_time_step(self):
        self.prep_system_for_iteration()

        # Update controllers
        for controller in self.controllers:
            controller.update_time_step(self.dt)

        # Update limb information
        for limb in self.limbs:
            limb.d1_old = limb.d1
            limb.d2_old = limb.d2
            limb.tangent_old = limb.tangent
            limb.ref_twist_old = limb.ref_twist

        # Update joint information similarly
        for joint in self.joints:
            joint.d1_old = joint.d1
            joint.d2_old = joint.d2
            joint.tangents_old = joint.tangents
            joint.ref_twist_old = joint.ref_twist