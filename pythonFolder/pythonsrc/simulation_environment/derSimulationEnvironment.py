

from abc import abstractmethod

from pythonsrc.globalDefinitions import RenderParams
from pythonsrc.logging.worldLogger import WorldLogger
from pythonsrc.world import world


class derSimulationEnvironment(object):
    #VARIABLES DECLARATIONS
    w_p : world = None
    logger_p : WorldLogger = None
    is_logging = False
    cmdline_per: int = 0
    def __init__(self, m_world, render_params : RenderParams, logger):
        self.w_p = m_world
        self.logger_p = logger
        if logger != None:
            print("HEYOOOO LOGGER")
            self.is_logging = True
        
        print("I AM RUNNING")

        print(self.logger_p)

        if self.is_logging:
            self.logger_p.world_ptr = self.w_p
            self.logger_p.setup()
        
        self.cmdline_per = render_params.cmd_line_per

    @abstractmethod
    def runSimulation():
        pass

    @abstractmethod
    def stepSimulation():
        pass

    def cmdlineOutputHelper(self):
        """
        Helper method to make command line output consistent across environments.
        """
        # print("RUNNING")
        self.cmdlineOutputHelperStatic(self.w_p, self.cmdline_per)

    @staticmethod
    def cmdlineOutputHelperStatic(s_world_p: world, s_cmdline_per):
        """
        Static helper method for command line output.
        This method is static to be compatible with OpenGL's C-based functions.
        :param s_world_p: The world object.
        :param s_cmdline_per: The command line verbosity frequency.
        """
        # print("RINININGHIGI")
        if s_cmdline_per == 0: return
        if s_world_p.get_time_step() % s_cmdline_per == 0:
            # print(s_world_p)
            s_world_p.print_sim_data()
    
    def cleanShutdown(self):
        """
        Handle a clean shutdown of the simulation environment.
        """
        self.__class__.cleanShutdownStatic(self.logger_p, self.is_logging)

    @staticmethod
    def cleanShutdownStatic(s_logger_p, s_is_logging):
        """
        Static version of clean shutdown for OpenGL compatibility.
        :param s_logger_p: Logger instance, if used.
        :param s_is_logging: Whether logging is enabled.
        """
        if s_is_logging and s_logger_p is not None:
            s_logger_p.flushLogs()
            print("Logging complete. Shutting down cleanly.")
        else:
            print("Shutting down cleanly without logging.")
