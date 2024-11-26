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
            # print("NOT SOLVED")
            self.prep_system_for_iteration()
            self.forces.compute_forces_and_jacobian(dt)
            # print(dt)

            # Compute norm of the force equationsS
            # print(self.force)
            # print("FORCE")
            # print(self.Force)
            normf = np.linalg.norm(self.Force)
            # print(self.iter)
            print("FORM", normf)

            if self.iter == 0:
                normf0 = normf

            # Force tolerance check
            # print("NORMYYY", normf)
            # print("NORM", normf0 * self.ftol)
            if normf <= normf0 * self.ftol:
                # print("NORMF")
                solved = True
                self.iter += 1
                continue
            
            # print(normf)

            # Adaptive time stepping if enabled
            if self.adaptive_time_stepping and self.iter != 0 and self.iter % self.adaptive_time_stepping_threshold == 0:
                # print("ADAPTIVE")
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
            idx = 0
            for limb in self.limbs:
                # print("INDEX", idx)
                # print(self.dx)
                curr_dx = limb.update_newton_x(self.dx, self.offsets[idx], self.alpha)
                max_dx = max(max_dx, curr_dx)
                idx+=1

            print("MAX DX:", max_dx)
            # print("dt:", dt)
            # print("dt:", self.dtol)

            # Dynamics tolerance check
            if max_dx / dt < self.dtol:
                solved = True
                self.iter += 1
                continue

            self.iter += 1

            # Exit if unable to converge
            # print(self.max_iter)
            if self.iter > self.max_iter:
                if self.terminate_at_max:
                    print(f"No convergence after {self.max_iter} iterations")
                    exit(1)
                else:
                    solved = True

        # print("ITER", self.iter)
        # print("LEAVING ")
        # print(dt)

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

        # print("FORCE: ")
        # print(self.Force.T)

        # Compute the initial slope
        q0 = 0.5 * np.power(np.linalg.norm(self.Force), 2)
        print("Q0: ", q0)
        # print("FORCE T VALUE", self.Force.T)
        # print("DX VALUES", self.DX)
        # print(self.Jacobian)
        dq0 = -np.dot(self.Force.T, np.dot(self.Jacobian, self.DX))#[0]

        print("DQ0:", dq0)

        success = False
        m2, m1 = 0.9, 0.1

        while not success:
            limb_idx = 0
            for joint in self.joints:
                joint.x = joint.x_ls.copy()
            for limb in self.limbs:
                limb.x = limb.x_ls.copy()
                limb.update_newton_x(self.dx, self.offsets[limb_idx], self.alpha)
                limb_idx+=1

            self.prep_system_for_iteration()
            self.forces.compute_force(dt)

            q = 0.5 * np.power(np.linalg.norm(self.Force), 2)
            print("Q VALUE: ", q)
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

        # print("ALPHA:", self.alpha)
        for limb in self.limbs:
            limb.x = limb.x_ls.copy()
            # print(limb.x)
            # print(limb.x)
        for joint in self.joints:
            joint.x = joint.x_ls.copy()

    def step_forward_in_time(self):
        # print("THIS IS FORWARD IN TIME")
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

        print("STEP FORWARD RUNNING IN TIME BACKWARD's Euler")

        # Update limb positions and velocities
        for limb in self.limbs:
            limb.u = (limb.x - limb.x0) / self.dt
            limb.x0 = limb.x

        self.update_system_for_next_time_step()
        return self.dt
    
    def update_system_for_next_time_step(self):
        # print("BACK WARD EULER RIGHT")
        self.prep_system_for_iteration()

        # Update controllers
        # print(self.controllers)
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