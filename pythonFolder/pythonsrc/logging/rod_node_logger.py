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

    # def __del__(self):
    #     """Destructor equivalent in Python, called when the object is deleted."""
    #     print("RodNodeLogger object is being deleted")

    def get_log_header(self):
        """
        Returns the header for the log file.
        :return: String header for the log file
        """
        return "Log Header: time, position, velocity"

    def get_log_data(self):
        """
        Returns the data to be logged.
        :return: String data for the log file
        """
        # In a real scenario, this would gather actual data.
        return "Log Data: 0.0, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]"