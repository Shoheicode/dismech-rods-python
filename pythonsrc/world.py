from pythonsrc.globalDefinitions import SimParams
from pythonsrc.rod_mechanics.force_container import forceContainer
from pythonsrc.rod_mechanics.soft_robots import softRobots


class world:
    def __init__(self, soft_robots: softRobots, forces: forceContainer, simParams: SimParams) -> None:
        self.soft_robots = soft_robots
        self.forces = forces
        self.time_step = 0
        self.currTime = 0
        self.total_time = simParams.sim_time

        self.forces.addForce(None)
        pass

    def update_time_step():
        pass

    def get_coordinate(i, limb_idx):
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

    