from wpilib.command import Command
from wpilib.command import CommandGroup
from wpilib import SmartDashboard
from subsystems import drive, climber
import oi

class LiftCommand(Command):
  """
  Base class used for command that manipulate climber legs.
  
  NOTE: All lift commands that extend this base class should only need
  to provide an execute() method.
  """

  def __init__(self, name):
    super().__init__(name)
    self.requires(climber)
    # NOTE: We may not use it, but I think we want full control
    # of the drive for all lift operations (we don't want operator
    # messing with drive wheels)
    self.requires(drive)
    self.debug = True

    # Following settings can be adjusted on dashbard if debug is set to true
    self.extendSpeed: float = 0.9
    self.retractSpeed: float = -0.9
    self.wheelPower: float = 0.75
    self.maxLeanUp: float = 2.0
    self.maxLeanDown: float = 4.0
    self.configure()

  def configure(self):
    if self.debug:
      # If still debugging/testing climber, allow getting values from dashboard
      self.extendSpeed: float = oi.OI.initializeNumber("Climb Extend Power", self.extendSpeed)
      # Hmmm, should we just show it as a negative number on the dashboard
      self.retractSpeed: float = -oi.OI.initializeNumber("Climb Retract Power", -self.retractSpeed)
      self.wheelPower: float = oi.OI.initializeNumber("Climb Wheel Power", self.wheelPower)
      self.maxLeanUp: float = oi.OI.initializeNumber("Climb Max Lean Up", self.maxLeanUp)
      self.maxLeanDown: float = oi.OI.initializeNumber("Climb Max Lean Down", self.maxLeanDown)

  def initialize(self):
    self.configure()

  def end(self):
    """ Stops all leg and wheel motors when any climber command terminates. """
    climber.stopAll()

  def interrupted(self):
    self.end()

  def getLean(self) -> float:
    """
    Returns degrees that robot is leaning forward (-) or backward (+).
    """
    climber.getLean()

  def isLeaningForward(self, lean) -> bool:
    """
    Indicates whether robot lean value is forward or backward.
    : param lean : Signed lean value.
    : return : True if leaning forward, false if leaning backward.
    """
    return lean < 0

  def reducePower(self, power, lean) -> float:
    """
    Returns a reduced power value based on amount of lean.
    : param power : Desired power value if level.
    : return : Reduced power level based on amount of lean.
    """
    # Reduce power by 0.05 per degree of lean, but not more
    # than 25 percent
    reduction = min(0.05 * lean, 0.25)
    return power * (1.0 - reduction)

  def fullyExtendBothLegs(self):
    """
    Tries to apply power values to smoothly
    extend both legs while maintaining a level robot.
    """
    maxLean: float = self.maxLeanUp
    frontPower: float = self.extendSpeed
    backPower: float = self.extendSpeed
    lean: float = self.getLean()

    if self.isLeaningForward(lean):
      backPower = self.reducePower(backPower, lean)
    else:
      frontPower = self.reducePower(frontPower, lean)

    # Apply computed power
    climber.moveFrontLegs(frontPower, maxLean)
    climber.moveBackLegs(backPower, maxLean)

  def maintainBackLegs(self):
    """
    Checks to see how out of "level" we are and if necessary will apply power
    to back motors to adjust.
    """
    # Note, I think we prefer leaning forwards slightly when
    # robot is up and moving onto ramp, so desired lean
    # might be better set to something less than 0.
    desiredLean: float = -2.0
    offLean: float = 1.0 # Turn off back motor if in this range
    maxLean: float = self.maxLeanDown # Turn on back motor if out of this range
    forwardLeanLimit = desiredLean - maxLean
    backwardLeanLimit = desiredLean + maxLean
    lean: float = self.getLean()

    if lean < -forwardLeanLimit:
      # Leaning too far forward, need to retract back legs
      climber.moveBackLegs(self.retractSpeed, -forwardLeanLimit)
    elif lean > backwardLeanLimit:
      # Leaning too far back, need to extend back legs
      climber.moveBackLegs(self.extendSpeed, backwardLeanLimit)
    elif lean > (desiredLean - offLean) and lean < (desiredLean + offLean):
      # Back in good range, turn motors off until something changes a lot
      climber.stopBack()


class ExtendBothLegs(LiftCommand):
    """ Command that lifts the robot into the air by extending both legs. """
    def __init__(self):
        super().__init__("ExtendBothLegs")

    def execute(self):
        self.fullyExtendBothLegs()

    def isFinished(self):
        return climber.isFullyExtendedBoth()


class RetractFrontLegs(LiftCommand):
    """ Command that retracts the front legs once front wheels are on top platform. """
    def __init__(self):
        super().__init__("RetractFrontLegs")

    def execute(self):
        # Keep back legs level
        self.maintainBackLegs()

        # Retract front legs
        maxLean = self.maxLeanDown
        climber.moveFrontLegs(self.retractSpeed, maxLean)

    def isFinished(self):
        # Hmmm, should we stop if sensor is not indicating that front
        # is over ground?
        return climber.isFullyRetractedFront() or not climber.isFrontOverGround()


class RetractBackLegs(LiftCommand):
    """ Command that retracts the back legs once drive wheels are on top platform. """
    def __init__(self):
        super().__init__("RetractBackLegs")

    def execute(self):
        # Retract both legs (keep them up)
        maxLean = self.maxLeanDown
        climber.moveBackLegs(self.retractSpeed, maxLean)
        climber.moveFrontLegs(self.retractSpeed, maxLean)

    def isFinished(self):
        # Hmmm, should we stop if sensor is not indicating that back
        # is over ground?
        return climber.isFullyRetractedBack() or not climber.isBackOverGround()


class DriveToFrontSensor(LiftCommand):
    """ Command that drives forward until the front sensor is over ground. """
    def __init__(self):
        super().__init__("DriveToFrontSensor")

    def execute(self):
        climber.setWheelPower(self.wheelPower)
        # Keep robot level with back legs while driving forward
        self.maintainBackLegs()
        # Should we also be driving main drive wheels?
        # Maybe minimal power so climber wheels don't have to fight
        drive.setPower(0.15, 0.15)

    def isFinished(self):
        return climber.isFrontOverGround()


class DriveToBackSensor(DriveToFrontSensor):
    """ Command that drives forward until the back sensor is over ground. """
    def __init__(self):
        super().__init__()
        self.setName("DriveToBackSensor")

    def isFinished(self):
        return climber.isBackOverGround()


class ClimbUp(CommandGroup):
    """
    Command that takes robot from facing podium all the way
    so that it is on top the podium.
    """
    def __init__(self):
        self.addSequential(ExtendBothLegs())
        self.addSequential(DriveToFrontSensor())
        self.addSequential(RetractFrontLegs())
        self.addSequential(DriveToBackSensor())
        self.addSequential(RetractBackLegs())
        # TODO: Determine command to drive rest of way
