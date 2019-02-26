import wpilib
from wpilib.command import Command
from wpilib.command import Subsystem
from wpilib.smartdashboard import SmartDashboard
import commandbased

from commands.performance import Performance

class Drive(wpilib.command.subsystem.Subsystem):
  """
  Minimal drive with two PWM motor controllers.
  """
  leftMotor: wpilib.PWMSpeedController = None
  rightMotor: wpilib.PWMSpeedController = None

  def __init__(self, leftMotor: wpilib.PWMSpeedController, rightMotor: wpilib.PWMSpeedController):
    super().__init__("Drive " + str(leftMotor.getChannel()) + "," + str(rightMotor.getChannel()))
    self.leftMotor = leftMotor
    self.rightMotor = rightMotor

  def setPower(self, left, right):
    self.leftMotor.set(left)
    self.rightMotor.set(right)

class DrivePower(wpilib.command.Command):
  """ Minimal command to control drive subsystem. """

  def __init__(self, drive: Drive, leftPower: float, rightPower: float):
    super().__init__("DrivePower " + str(drive.leftMotor.getChannel()) + "," + str(drive.rightMotor.getChannel()))
    self.requires(drive)
    self.leftPower: float = leftPower
    self.rightPower: float = rightPower
    self.drive: Drive = drive

  def execute(self):
    self.drive.setPower(self.leftPower, self.rightPower)

  def isFinished(self):
    return False

  def interrupted(self):
    self.drive.setPower(0, 0)

  def end(self):
    self.interrupted()


class MyRobot(commandbased.CommandBasedRobot):
  """
  Minimal implementation of a robot program using the command based framework.
  """
  # Set to number of subsystems to create (set to 0 for no subsystems, 10 is max)
  numSubsystems: int = 1

  # Used to track performance
  performance: Performance = None

  def robotInit(self):
    """ Initalizes all subsystems and user controls. """
    if self.numSubsystems > 0:
      # Create drive subsystems
      for i in range(0, self.numSubsystems):
        pwm: int = i * 2
        leftMotor = wpilib.VictorSP(pwm)
        rightMotor = wpilib.VictorSP(pwm + 1)
        drive = Drive(leftMotor, rightMotor)
        SmartDashboard.putData("Forward " + str(i), DrivePower(drive, 0.2, 0.2))
        SmartDashboard.putData("Backward " + str(i), DrivePower(drive, -0.2, -0.2))

    # Add command to dashboard to track time for one periodic pass
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
