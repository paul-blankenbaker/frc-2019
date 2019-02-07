import wpilib
from wpilib.smartdashboard import SmartDashboard
import commandbased
import oi
import subsystems
from commands.performance import Performance

class MyRobot(commandbased.CommandBasedRobot):

    def robotInit(self):
        """ Initalizes all subsystems and user controls. """
        # Set up subsystems
        subsystems.initialize()
        # Set up user controls
        oi.initialize()
        self.debug = True

        if self.debug:
            self.performance = Performance()
            SmartDashboard.putData("Measure Performance", self.performance)

    def loopFunc(self):
        """ Override base implementation so we can peek at how long each iteration takes. """
        super().loopFunc()
        # Record how long it took to run iteration of loop
        if self.performance != None:
            self.performance.updateRunTime(self.watchdog.getTime())

if __name__ == "__main__":
    wpilib.run(MyRobot)
