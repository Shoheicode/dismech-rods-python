from io import StringIO
from pythonsrc.logging.worldLogger import WorldLogger


class RodNodeLogger(WorldLogger):
    def setup_helper(self):
        pass  # Abstract method to be implemented in the subclass
    def __init__(self, logfile_base, logfile_suffix=None, logging_output_file=None, logging_period=1):
        """
        Initializes the RodNodeLogger class with parameters.

        :param logfile_base: The base name of the log file
        :param logfile_suffix: The suffix of the log file (optional)
        :param logging_output_file: The output file object for logging
        :param logging_period: Period for logging
        """
        super().__init__("node", logfile_base, logging_output_file, logging_period)
        self.logfile_base = logfile_base
        self.logfile_suffix = logfile_suffix
        self.logging_output_file = logging_output_file
        self.logging_period = logging_period
        # self.world_ptr = None

    # def __del__(self):
    #     """Destructor equivalent in Python, called when the object is deleted."""
    #     ("RodNodeLogger object is being deleted")

    def get_log_header(self):
        """
        Returns the header for the log file.
        :return: String header for the log file
        """
        return ""

    def get_log_data(self):
        """
        Returns the data to be logged.
        :return: String data for the log file
        """
        print("HEYOddfdfd")
        # In a real scenario, this would gather actual data.
        log_data = StringIO()
        log_data.write(f"{self.world_ptr.get_current_time()}")
        
        for limb in self.world_ptr.soft_robots.limbs:
            for i in range(limb.nv):
                v = limb.get_vertex(i)
                log_data.write(f",{v[0]},{v[1]},{v[2]}")
                
        return log_data.getvalue()