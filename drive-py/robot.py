import wpilib
from wpilib.command import Command
from wpilib.command import Subsystem
from wpilib.smartdashboard import SmartDashboard
import commandbased
import oi
import subsystems
from commands.performance import Performance

class MyRobot(commandbased.CommandBasedRobot):
  # Set to False to remove diagnostic tools
  debug: bool = True

  # Set to True to disable ALL subsystems and operator interface
  bareBones: bool = True

  def robotInit(self):
    """ Initalizes all subsystems and user controls. """
    if not self.bareBones:
      # Set up subsystems
      subsystems.initialize()
      # Set up user controls
      oi.initialize()

    if self.debug:
      self.performance = Performance()
      SmartDashboard.putData("Measure Performance", self.performance)

  def autonomousInit(self):
    if not self.bareBones:
      autonCommand = oi.instance.getSelectedAuton()
      autonCommand.start()

  def disabledInit(self):
      """ Cancel current commands running on each subsystem - force back to defaults. """
      if not self.bareBones:
        list = [subsystems.drive]
        for s in list:
          c: Command = s.getCurrentCommand()
          if c != None:
            c.cancel()

  def loopFunc(self):
    """ Override base implementation so we can peek at how long each iteration takes. """
    super().loopFunc()
    # Record how long it took to run iteration of loop
    if self.performance != None:
      self.performance.updateRunTime(self.watchdog.getTime())


if __name__ == "__main__":
    wpilib.run(MyRobot)
