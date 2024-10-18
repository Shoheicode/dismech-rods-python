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
        pass

    def get_M1(i, limb_idx):
        pass

    def get_M2(i, limb_idx):
        pass

    def get_current_time():
        pass

    def simulation_runing():
        pass

    def get_time_step():
        pass

    def print_sim_data():
        pass

    def floor_exists():
        pass

    def get_floor_Z():
        pass

    def update_cons(self):
        for limb in self.soft_robots.limbs:
            limb.update_map()
        self.stepper.update()

    