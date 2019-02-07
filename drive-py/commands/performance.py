
import wpilib
from wpilib.smartdashboard import SmartDashboard

# TODO: Move Stats to utility package
class Stats():
    """ Helper class to track statistics (min, max, cnt, sum, avg). """
    def __init__(self):
        self.zero()

    def zero(self):
        self.__min : float = 0
        self.__max : float = 0
        self.__cnt : int = 0
        self.__sum : float = 0

    def getMin(self) -> float:
        return self.__min

    def getMax(self) -> float:
        return self.__max

    def getCount(self) -> int:
        return self.__cnt

    def getSum(self) -> float:
        return self.__sum

    def getAvg(self) -> float:
        avg: float = 0
        if self.__cnt > 0:
            avg = self.__sum / self.__cnt
        return avg

    def add(self, val: float):
        if self.__cnt == 0:
            self.__min = self.__max = val
        elif val > self.__max:
            self.__max = val
        elif val < self.__min:
            self.__min = val
        self.__cnt += 1
        self.__sum += val


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

