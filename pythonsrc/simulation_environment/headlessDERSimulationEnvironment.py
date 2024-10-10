from pythonsrc.globalDefinitions import RenderParams
from pythonsrc.logging.worldLogger import WorldLogger
from pythonsrc.simulation_environment.derSimulationEnvironment import derSimulationEnvironment
from pythonsrc.world import world


class HeadlessDERSimulationEnvironment(derSimulationEnvironment):
    def __init__(self, m_world: world, render_params : RenderParams, logger: WorldLogger):
        super().__init__(m_world, render_params, logger)

    def stepSimulation():
        pass

    def runSimulation():
        pass
