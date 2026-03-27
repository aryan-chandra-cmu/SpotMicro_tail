# Written by team Tailenders (Atharva Sunder, Kausik Kolluri, Jash Lapsiwala, Aryan Chandra, Raymond Cao)
# with assistance from Claude Opus 4.6
# Date: 25th March 2026

"""
Data logger for the Spot Micro quadruped.

Provides a simple interface for logging numerical data during
control loops and saving it to CSV files.
"""

import csv
import os
from datetime import datetime


class DataLogger:
    """
    Simple columnar data logger.

    Usage:
        logger = DataLogger(headers=["time", "x", "y", "z"])
        for t in ...:
            logger.log(t, x, y, z)
        logger.save("output.csv")
    """

    def __init__(self, headers):
        """
        Initialize the DataLogger.

        Parameters
        ----------
        headers : list of str
            Column headers for the logged data.
        """
        self.headers = headers
        self.num_columns = len(headers)
        self.data = []

    def log(self, *values): # '*' means Accept ANY number of positional arguments and pack them into a tuple called values.
        """
        Log a single row of data.

        Parameters
        ----------
        *values : float
            Values to log, must match the number of headers.

        Raises
        ------
        ValueError
            If the number of values does not match the number of headers.
        """
        if len(values) != self.num_columns:
            raise ValueError(
                f"Expected {self.num_columns} values, got {len(values)}. "
                f"Headers: {self.headers}"
            )
        self.data.append(list(values))

    def save(self, filepath=None, directory="logs"):
        """
        Save logged data to a CSV file.

        Parameters
        ----------
        filepath : str, optional
            Full path to save the CSV file. If None, a timestamped
            filename is generated in the specified directory.
        directory : str, optional
            Directory to save the file in (default: 'logs').
            Only used if filepath is None.

        Returns
        -------
        str
            The path of the saved file.
        """
        if filepath is None:
            os.makedirs(directory, exist_ok=True)
            timestamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
            filepath = os.path.join(directory, f"ik_log_{timestamp}.csv")

        with open(filepath, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(self.headers)
            writer.writerows(self.data)

        print(f"Data saved to {filepath} ({len(self.data)} rows)")
        return filepath
