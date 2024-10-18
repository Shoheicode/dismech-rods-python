from pythonFolder.pythonsrc.rod_mechanics.inner_forces.elastic_bending_force import ElasticBendingForce
from pythonFolder.pythonsrc.rod_mechanics.inner_forces.elastic_streching_force import ElasticStretchingForce
from pythonFolder.pythonsrc.rod_mechanics.inner_forces.elastic_twisting_force import ElasticTwistingForce
from pythonFolder.pythonsrc.rod_mechanics.inner_forces.inertial_force import InertialForce
from pythonFolder.pythonsrc.time_stepper.basetimestepper import BaseTimeStepper
from pythonsrc.globalDefinitions import IntegratorMethod, SimParams
from pythonsrc.rod_mechanics.force_container import ForceContainer
from pythonsrc.rod_mechanics.soft_robots import SoftRobots


class world:
    def __init__(self, soft_robots: SoftRobots, forces: ForceContainer, sim_params: SimParams) -> None:
        self.soft_robots = soft_robots
        self.forces = forces
        self.time_step = 0
        self.curr_time = 0
        self.total_time = sim_params.sim_time
        self.stepper:BaseTimeStepper= None

        self.forces.add_force(ElasticStretchingForce(soft_robots))
        self.forces.add_force(ElasticBendingForce(soft_robots))
        self.forces.add_force(ElasticTwistingForce(soft_robots))

        if sim_params.integrator not in [IntegratorMethod.FORWARD_EULER, IntegratorMethod.VERLET_POSITION]:
            self.forces.add_force(InertialForce(soft_robots))

        self.stepper.init_stepper()

        if sim_params.enable_2d_sim:
            for limb in soft_robots.limbs:
                limb.enable_2d_sim()

        self.update_cons()

        self.stepper.update_system_for_next_time_step()

    def update_time_step(self):
        self.curr_time += self.stepper.step_forward_in_time()
        self.time_step += 1

    def get_coordinate(self, i, limb_idx):
        return self.soft_robots.limbs[limb_idx].x[i]

    def get_M1(self, i, limb_idx):
        return self.soft_robots.limbs[limb_idx].m1[i, :]

    def get_M2(self, i, limb_idx):
        return self.soft_robots.limbs[limb_idx].m2[i, :]

    def get_current_time():
        pass

    def simulation_runing(self):
        if self.curr_time < self.total_time:
            return True
        else:
            print("Completed simulation.")
            return False

    def get_time_step():
        pass

    def print_sim_data(self):
        cf = self.forces.cf
        ff = self.forces.ff

        if cf and ff:
            if cf.get_num_collisions() > 0:
                print(f"time: {self.curr_time:.4f} | iters: {self.stepper.iter} | "
                      f"con: {cf.get_num_collisions()} | min_dist: {cf.get_min_dist():.6f} | "
                      f"floor_con: {ff.num_contacts} | f_min_dist: {ff.min_dist:.6f}")
            else:
                print(f"time: {self.curr_time:.4f} | iters: {self.stepper.iter} | con: 0 | "
                      f"min_dist: N/A | floor_con: {ff.num_contacts} | f_min_dist: {ff.min_dist:.6f}")
        elif cf:
            if cf.get_num_collisions() > 0:
                print(f"time: {self.curr_time:.4f} | iters: {self.stepper.iter} | "
                      f"con: {cf.get_num_collisions()} | min_dist: {cf.get_min_dist():.6f}")
            else:
                print(f"time: {self.curr_time:.4f} | iters: {self.stepper.iter} | con: 0 | min_dist: N/A")
        elif ff:
            print(f"time: {self.curr_time:.4f} | iters: {self.stepper.iter} | "
                  f"floor_con: {ff.num_contacts} | f_min_dist: {ff.min_dist:.6f}")
        else:
            print(f"time: {self.curr_time:.4f} | iters: {self.stepper.iter}")

    def floor_exists():
        pass

    def get_floor_Z():
        pass

    def update_cons(self):
        for limb in self.soft_robots.limbs:
            limb.update_map()
        self.stepper.update()

    