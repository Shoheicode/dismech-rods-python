from pythonsrc.globalDefinitions import RenderParams
from pythonsrc.logging.worldLogger import WorldLogger
from pythonsrc.simulation_environment.derSimulationEnvironment import derSimulationEnvironment
from pythonsrc.world import world


class HeadlessDERSimulationEnvironment(derSimulationEnvironment):
    def __init__(self, m_world: world, render_params : RenderParams, logger: WorldLogger):
        super().__init__(m_world, render_params, logger)

    def stepSimulation():
        try:
            super().w_p.update_time_step()
        except RuntimeError as e:
            print("Caught a runtime error when trying to world->updateTimeStep: ", e)
            print("Attempting clean shutdown")


    def runSimulation():
        pass
