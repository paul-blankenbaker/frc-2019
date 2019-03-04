import wpilib
from wpilib import XboxController
from wpilib import SpeedControllerGroup
from wpilib.command import Command
from wpilib.command import Subsystem
from wpilib.drive import DifferentialDrive
from wpilib.interfaces import GenericHID
from wpilib.interfaces import SpeedController
from wpilib.smartdashboard import SmartDashboard
import commandbased

from commands.performance import Performance

class Drive(wpilib.command.subsystem.Subsystem):
  """
  Minimal drive with two PWM motor controllers.
  """
  leftMotor: SpeedController = None
  rightMotor: SpeedController = None

  def __init__(self, id: int, leftMotor: SpeedController, rightMotor: SpeedController):
    super().__init__("Drive")
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

class DriveArcade(wpilib.command.Command):
  """ Command to control drive using gamepad in arcade mode. """
  diffDrive: DifferentialDrive
  gamePad: GenericHID
  nominal: float

  def __init__(self, drive: Drive, leftMotors: SpeedControllerGroup, rightMotors: SpeedControllerGroup, gamePad: GenericHID):
    super().__init__("DriveArcade")
    self.requires(drive)
    self.gamePad = gamePad
    self.diffDrive = DifferentialDrive(leftMotors, rightMotors)
    self.nominal = 3.0 / 8.0
    self.deadband = 0.15 # XBox360 controller has a lot of slop

  def tweak(self, val, scale):
    tweaked: float = DifferentialDrive.applyDeadband(val, self.deadband)
    if tweaked == 0.0:
      # User input value in deadband, just return 0.0
      return tweaked
    tweaked *= scale * (1 - self.nominal)
    if tweaked > 0:
      tweaked = (tweaked * tweaked) + self.nominal
    else:
      tweaked = (tweaked * -tweaked) - self.nominal
    return tweaked

  def initialize(self):
    self.diffDrive.setSafetyEnabled(True)

  def execute(self):
    throttle: float = self.tweak(-self.gamePad.getY(GenericHID.Hand.kLeft), 1.0)
    rotation: float = self.tweak(self.gamePad.getX(GenericHID.Hand.kRight), 0.5)
    #print("Throttle", throttle, "Rotation", rotation)
    self.diffDrive.arcadeDrive(throttle, rotation, False)

  def isFinished(self):
    return False

  def interrupted(self):
    self.diffDrive.stopMotor()
    self.diffDrive.setSafetyEnabled(False)

  def end(self):
    self.interrupted()


class MyRobot(commandbased.CommandBasedRobot):
  """
  Minimal implementation of a robot program using the command based framework.
  """
  
  # Set to True to enable arcade drive subsystem and command using gamepad where
  # PWM 0, 1 go with left side and PWM 2, 3 go with right side
  enableDrive: bool = True

  # Set to number of additional PWM subsystems to create (set to 0 for no extra subsystems,
  # 8 is max if drive enabled, 10 if not)
  numSubsystems: int = 1

  # Used to track performance
  performance: Performance = None

  def robotInit(self):
    """ Initalizes all subsystems and user controls. """
    if self.numSubsystems > 0:
      # Create drive subsystems
      pwmOfs: int = 0
      if self.enableDrive:
        leftMotors = SpeedControllerGroup(wpilib.VictorSP(0), wpilib.VictorSP(1))
        rightMotors = SpeedControllerGroup(wpilib.VictorSP(2), wpilib.VictorSP(3))
        gamePad: GenericHID = XboxController(0)
        drive: Drive = Drive(99, leftMotors, rightMotors)
        drive.setDefaultCommand(DriveArcade(drive, leftMotors, rightMotors, gamePad))
        pwmOfs += 4

      for i in range(0, self.numSubsystems):
        pwm: int = pwmOfs + i * 2
        leftMotor: SpeedController = wpilib.VictorSP(pwm)
        rightMotor: SpeedController = wpilib.VictorSP(pwm + 1)
        drive = Drive(i, leftMotor, rightMotor)
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
