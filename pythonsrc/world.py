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

    def updateTimeStep():
        pass

    def getCoordinate(i, limb_idx):
        pass

    def getM1(i, limb_idx):
        pass

    def getM2(i, limb_idx):
        pass

    def getCurrentTime():
        pass

    def simulationRuning():
        pass

    def getTimeStep():
        pass

    def printSimData():
        pass

    def floorExists():
        pass

    def getFloorZ():
        pass

    