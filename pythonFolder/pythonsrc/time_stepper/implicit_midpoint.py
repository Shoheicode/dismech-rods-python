from pythonsrc.globalDefinitions import SimParams
from pythonsrc.rod_mechanics.force_container import ForceContainer
from pythonsrc.solvers.solver_types import SolverType
from pythonsrc.rod_mechanics.soft_robots import SoftRobots
from pythonsrc.time_stepper.backward_euler import BackwardEuler


class ImplicitMidpoint(BackwardEuler):
    def __init__(self, soft_robots: SoftRobots, forces: ForceContainer, sim_params: SimParams, solver_type: str):
        super().__init__(soft_robots, forces, sim_params, solver_type)

    def step_forward_in_time(self):
        """
        This method implements the specific time-stepping algorithm for the implicit midpoint.
        It should use the midpoint of the interval to compute the next time step.
        """
        # print("FORWARD IN TIME")
        # Placeholder for the time-stepping implementation
        # Specific implicit midpoint step logic should be implemented here
        # Set initial time step
        self.dt = self.orig_dt

        # Initial guess using last solution
        for limb in self.limbs:
            limb.update_guess(0.01, 0.5 * self.dt)

        # print(self.forces.cf)

        # Perform collision detection if contact is enabled
        if self.forces.cf:
            self.forces.cf.broad_phase_collision_detection()

        # print("NEWTON METHOD")

        # Compute position at T = t + 0.5 * dt
        self.dt = 2 * self.newton_method(0.5 * self.dt)

        for limb in self.limbs:
            # Compute velocity at T = t + 0.5 * dt
            limb.u = (limb.x - limb.x0) / (0.5 * self.dt)

            # Compute position at T = t + dt
            limb.x = 2 * limb.x - limb.x0
            # for i in range(limb.ne):
            #     # print("X POSITION", i, ":", limb.x[i*4])
            #     # print("Y POSITION", i, ":", limb.x[i*4+1])
            #     print("Z POSITION", i, ":", limb.x[i*4+2])
            # limb.x0 = limb.x

            # Compute velocity at T = t + dt
            limb.u = 2 * limb.u - limb.u0
            # print("U POSITION: ", limb.u)
            limb.u0 = limb.u

        # Update the system for the next time step
        self.update_system_for_next_time_step()

        return self.dt
