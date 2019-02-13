
import wpilib
from wpilib.smartdashboard import SmartDashboard
from math5511.stats import Stats

class Performance(wpilib.command.Command):
    """ Used to monitor how long it takes for code to run (helper class for main robot.py class). """
    def __init__(self):
        super().__init__("Performance")
        self.stats: Stats = Stats()
        self.last: float = 0
        self.setRunWhenDisabled(True)

    def updateRunTime(self, timeSecs: float):
        """ This is method is called externally by the class that is monitoring run time.

        : param timeSecs : How long the last run to (in seconds)
        """
        self.last = timeSecs * 1000.0
        self.stats.add(self.last)

    def initialize(self):
        """ Reset the statistics each time this command is started. """
        self.last = 0
        self.stats.zero()

    def execute(self):
        """ Show accumulated statistics that we care about. """
        SmartDashboard.putNumber("Run Last", self.last)
        SmartDashboard.putNumber("Run Avg", self.stats.getAvg())
        SmartDashboard.putNumber("Run Max", self.stats.getMax())

    def isFinished(self):
        return False

