import os
import datetime
from abc import ABC, abstractmethod

# Abstract class in Python for worldLogger
class WorldLogger(ABC):
    def __init__(self, file_name_prefix: str, logfile_base: str, data_file, period: int):
        self.file_name_prefix = file_name_prefix
        self.logfile_base = logfile_base
        self.m_data_file = data_file  # file object
        self.period = period
        self.num_lines_header = 0
        self.world_ptr = None
        self.m_fileName = ""
    
    def setup(self):
        """
        Setup function to initialize tasks that are prevented from taking place before construction.
        Calls a helper that can be overridden in child classes.
        """
        self.init_log_file()
        self.setup_helper()

    def init_log_file(self):
        """
        Initializes the log file.
        """
        timestamp = self.get_timestamp()
        log_folder = self.get_time_date_folder_path()
        
        # Ensure the log folder exists
        os.makedirs(log_folder, exist_ok=True)
        
        # Set the full file path
        self.m_fileName = os.path.join(log_folder, f"{self.file_name_prefix}_{timestamp}.log")
        self.m_data_file = open(self.m_fileName, 'w')  # Open the log file for writing
    
    def prune_empty_log(self):
        """
        Clean shutdown, remove a log file that has no data.
        """
        if self.count_lines_in_log() <= self.num_lines_header:
            # Close and remove the empty log file
            self.m_data_file.close()
            os.remove(self.m_fileName)

    def count_lines_in_log(self):
        """
        Count the number of lines in the log file.
        """
        with open(self.m_fileName, 'r') as f:
            return sum(1 for line in f)

    def log_world_data(self):
        """
        Log world data to the file, at a periodic rate.
        """
        if self.world_ptr:  # Assuming world_ptr is set up
            data = self.get_log_data()
            self.m_data_file.write(data + "\n")

    def get_timestamp(self) -> str:
        """
        Get a timestamp string in the format YearMonthDay_HourMinuteSecond.
        """
        return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    def get_time_date_folder_path(self) -> str:
        """
        Generate folder path based on the current date and time.
        Example: "2020/2020_05/2020_05_07/2020_05_07_2"
        """
        now = datetime.datetime.now()
        year = now.strftime("%Y")
        month = now.strftime("%Y_%m")
        day = now.strftime("%Y_%m_%d")
        hour = str(now.hour)
        return os.path.join(year, month, f"{day}_{hour}")

    @abstractmethod
    def get_log_header(self) -> str:
        """
        Abstract method to return the header for this logger.
        Must be implemented by child classes.
        """
        pass

    @abstractmethod
    def get_log_data(self) -> str:
        """
        Abstract method to return the data to log.
        Must be implemented by child classes.
        """
        pass

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
        if not self.m_data_file.closed:
            self.m_data_file.close()
