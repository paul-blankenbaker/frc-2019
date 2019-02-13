
import wpilib
import math
from wpilib.smartdashboard import SmartDashboard
from math5511.stats import Stats

class LoadTest(wpilib.command.Command):
    """ Runs a set of operations to test performance. """
    def __init__(self):
        super().__init__("LoadTest")
        self.previousBelow: bool = False
        self.loopCnts: int = 8096
        self.runSecs: float = 0
        self.timer: wpilib.Timer = wpilib.Timer()
        self.stats: Stats = Stats()
        self.setRunWhenDisabled(True)

    def runTest(self, cnt) -> float:
        accum: float = 0.0
        for i in range(0, cnt):
            n: int = int(i / 4)
            for j in range(0, n):
                x = int((int(j >> 3) + 1) * 7 / 8)
                if j % 4:
                    x = -x
                self.stats.add(x)

            ratio = float(i) / cnt
            accum += math.sin(ratio)
            accum += math.pow(math.pi, ratio)
            accum += math.cos(ratio)
            self.stats.add(accum)

        return accum

    def initialize(self):
        """ Reset for next run. """
        self.previousBelow = False
        self.runSecs = 0
        self.loopCnts = 1
        self.stats.zero()

    def execute(self):
        """ Measure how long it takes to run a bunch of iterations. """
        SmartDashboard.putNumber("CPU Test Loops", self.loopCnts)
        self.timer.reset()
        self.timer.start()
        self.runTest(self.loopCnts)
        self.runSecs = self.timer.get()

    def isFinished(self):
        """ Keep running until we cross over from too short of time to too long of time. """
        if self.runSecs > 0.1:
            if self.previousBelow:
                return True
            self.previousBelow = False
            self.loopCnts = int(self.loopCnts / 2)
        else:
            self.previousBelow = True
            self.loopCnts = int(self.loopCnts * 2)
        return False

    def end(self):
        hz = 0
        if self.runSecs > 0:
            hz = self.loopCnts / self.runSecs
        SmartDashboard.putNumber("CPU Test Secs", self.runSecs)
        SmartDashboard.putNumber("CPU Test Hz", hz)

