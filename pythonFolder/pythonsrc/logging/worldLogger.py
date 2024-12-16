import os
import time
from abc import ABC, abstractmethod

from pythonsrc.world import world

# Abstract class in Python for worldLogger
class WorldLogger(ABC):
    @abstractmethod
    def setup_helper(self):
        pass  # Abstract method to be implemented in the subclass
    def __init__(self, file_name_prefix: str, logfile_base: str, data_file, period: int):
        self.m_data_file = data_file  # file object
        self.period = period
        self.num_lines_header = 0
        self.file_name_prefix = file_name_prefix
        self.world_ptr = None

        if self.file_name_prefix == "":
            raise ValueError("Must specify a prefix for the worldLogger file name!")
        self.logfile_base = logfile_base
        if self.logfile_base == "":
            raise ValueError("Must specify the folder to be used for logging via logfile-base in options.txt!")
        
        if (self.logfile_base[0] == '~'):
            home = os.getenv("HOME")

            # Remove the tilde from the beginning of the string
            logfile_base = logfile_base[1:]
            # Concatenate the home directory with the remaining part of logfile_base
            logfile_base = os.path.join(home, logfile_base)
        
        # Assuming `logfile_base`, `file_name_prefix`, and `verbosity` are defined
        logfile_base = logfile_base.rstrip('/') + '/'

        # Expand out the folder path based on a nicely organized year, month, day, hour hierarchy (commented out in C++)
        # logfile_base = os.path.join(logfile_base, get_time_date_folder_path()) 
        # if verbosity >= 1:
        #     (f"Logging data in the folder {logfile_base}")

        # Create this folder if it does not already exist
        os.makedirs(logfile_base, exist_ok=True)

        # Save the file name here
        # Assume that we'll log to a file in the datafiles directory
        timestamp = time.strftime("%Y%m%d_%H%M%S")  # Equivalent of getTimestamp()
        file_name = f"{logfile_base}{file_name_prefix}_{timestamp}"

        # Finally, since it's possible that we will run multiple simulations within one second,
        # check if a previous iteration already created this file and append a 1, 2, 3, etc. on it
        if os.path.exists(f"{file_name}.csv"):
            # Find the next biggest counter to add
            repeated_file_num = 2
            while os.path.exists(f"{file_name}_{repeated_file_num}.csv"):
                repeated_file_num += 1
            # Tack on the counter
            file_name = f"{file_name}_{repeated_file_num}"

        # Append the .csv extension
        self.file_name = f"{file_name}.csv"

        # SOMEONE ELSE must init the log file, since that depends on the derived class!
        # init_log_file(p_world, file_name_prefix)
        
        # MAY NEED TO LOOK AT THIS
        # self.world_ptr = world(None, None, None)
        # self.m_fileName = ""
    
    def setup(self):
        """
        Setup function to initialize tasks that are prevented from taking place before construction.
        Calls a helper that can be overridden in child classes.
        """
        self.setup_helper()
        self.init_log_file()

    def init_log_file(self):
        """Helper function to create and initialize the log file."""
        with open(self.file_name, 'w') as f:
            f.write(self.get_log_header() + "\n")
        self.num_lines_header = self.count_lines_in_log()
    
    def prune_empty_log(self):
        """
        Clean shutdown, remove a log file that has no data.
        """
        (f"Checking if the log file {self.file_name} is empty...")
        min_useful_samples = 50
        if self.count_lines_in_log() - self.num_lines_header < min_useful_samples:
            ("Log file is almost empty, removing...")
            os.remove(self.file_name)
            ("File removal successful.")
        else:
            ("Log file contains data, not removing.")

    def count_lines_in_log(self):
        """
        Count the number of lines in the log file.
        """
        with open(self.file_name, 'r') as f:
            return sum(1 for _ in f)

    def log_world_data(self):
        """
        Log world data to the file, at a periodic rate.
        """
        print("DATA PRINTs")
        if self.world_ptr.get_time_step() % self.period == 0:
            with open(self.file_name, 'a') as f:
                f.write(self.get_log_data() + "\n")

    def get_timestamp(self) -> str:
        """
        Get a timestamp string in the format YearMonthDay_HourMinuteSecond.
        """
        return time.strftime("%m_%d_%H_%M_%S")

    def get_time_date_folder_path(self) -> str:
        """
        Generate folder path based on the current date and time.
        Example: "2020/2020_05/2020_05_07/2020_05_07_2"
        """
        return time.strftime("%Y_%m_%d_%H")

    @abstractmethod
    def get_log_header(self) -> str:
        """
        Abstract method to return the header for this logger.
        Must be implemented by child classes.
        """
        raise NotImplementedError

    @abstractmethod
    def get_log_data(self) -> str:
        """
        Abstract method to return the data to log.
        Must be implemented by child classes.
        """
        raise NotImplementedError

    @abstractmethod
    def setup_helper(self):
        """
        Setup helper, can be overridden by child classes.
        """
        pass

    def __del__(self):
        """
        Destructor to ensure the file is properly closed.
        """
        # if not self.m_data_file.closed:
        #     self.m_data_file.close()
