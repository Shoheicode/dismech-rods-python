from pythonsrc.globalDefinitions import RenderParams
from pythonsrc.logging.worldLogger import WorldLogger
from pythonsrc.simulation_environment.derSimulationEnvironment import derSimulationEnvironment
from pythonsrc.world import world


class HeadlessDERSimulationEnvironment(derSimulationEnvironment):
    def __init__(self, m_world: world, render_params : RenderParams, logger: WorldLogger):
        super().__init__(m_world, render_params, logger)

    def stepSimulation(self):
        try:
            print("RUNNINGT STEP SIMULATION CODE")
            print(self.w_p.stepper)
            self.w_p.update_time_step()
        except RuntimeError as e:
            print("Caught a runtime error when trying to world->updateTimeStep: ", e)
            print("Attempting clean shutdown")
            self.cleanShutdown()
            return
        
        print("ESCAPED THE FIRST STATEMENT")

        if self.is_logging:
            self.logger_p.log_world_data()
        
        self.cmdlineOutputHelper()

    def runSimulation(self):
        while self.w_p.simulation_runing():
            self.stepSimulation()
