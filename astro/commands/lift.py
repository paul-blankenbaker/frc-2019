from wpilib.command import Command
from wpilib.command import CommandGroup
from wpilib import SmartDashboard
import subsystems
from subsystems.climber import Climber, Leg
import oi

class LiftCommand(Command):
  """
  Base class used for command that manipulate climber legs.
  
  NOTE: All lift commands that extend this base class should only need
  to provide an execute() method.
  """

  # Set to True if you want lift commands to control the main drive
  # as well. Set to False if you want operator to be able to run
  # main drive train while lift commands are running.
  controlDrive: bool = True

  # Parts associated with climber subsystem
  climber: Climber
  frontLeg: Leg
  backLeg: Leg

  # Counts of extensions/retractions when command starts.
  # Used to detect when limit reached even if we move off sensor.
  backLegExtended: int = 0
  backLegRetracted: int = 0
  frontLegExtended: int = 0
  frontLegRetracted: int = 0

  def __init__(self, name: str):
    """
    Base constructor requires all subsystems - you MUST extend this class!
    : param name : Name for your implementing class.
    """
    super().__init__(name)
    self.requires(subsystems.climber)
    # NOTE: We may not use it, but I think we want full control
    # of the drive for all lift operations (we don't want operator
    # messing with drive wheels)
    if self.controlDrive:
      self.requires(subsystems.drive)
    self.debug = True

    self.climber = subsystems.climber
    self.frontLeg = self.climber.getFrontLeg()
    self.backLeg = self.climber.getBackLeg()

    # Following settings can be adjusted on dashbard if debug is set to true
    self.extendSpeed: float = 0.9
    self.retractSpeed: float = -0.9
    self.wheelPower: float = 0.75
    self.maxLeanUp: float = 2.0
    self.maxLeanDown: float = 4.0
    self.configure()

  def configure(self):
    """
    Reads/updates configurable settings from dashboard.
    """
    if self.debug:
      # If still debugging/testing climber, allow getting values from dashboard
      self.extendSpeed: float = oi.OI.initializeNumber("Climb Extend Power", self.extendSpeed)
      # Hmmm, should we just show it as a negative number on the dashboard
      self.retractSpeed: float = -oi.OI.initializeNumber("Climb Retract Power", -self.retractSpeed)
      self.wheelPower: float = oi.OI.initializeNumber("Climb Wheel Power", self.wheelPower)
      self.maxLeanUp: float = oi.OI.initializeNumber("Climb Max Lean Up", self.maxLeanUp)
      self.maxLeanDown: float = oi.OI.initializeNumber("Climb Max Lean Down", self.maxLeanDown)

  def initialize(self):
    """
    This should handle initialization for most cases, if you need to
    override this method, make sure you invoke this base implementation
    as well (super call).
    """
    # Get initial counts at leg end point sensors so we can detect
    # if sensor is tripped while command is running even if leg
    # moves off sensor
    self.frontLegExtended = self.frontLeg.getExtendedCount()
    self.frontLegRetracted = self.frontLeg.getRetractedCount()
    self.backLegExtended = self.backLeg.getExtendedCount()
    self.backLegRetracted = self.backLeg.getRetractedCount()
    # Load current user configuration from dashboard 
    self.configure()
    # Make sure main drive motors are off
    if self.controlDrive:
      subsystems.drive.stop()

  def end(self):
    """ Stops all leg and wheel motors when any climber command terminates. """
    subsystems.climber.stopAll()

  def interrupted(self):
    """ See end(). """
    self.end()

  def abort(self):
    """
    Will cancel this command or the command group it belongs too - use this if things go really BAD.
    """
    cg: CommandGroup = self.getGroup()
    cgLast: CommandGroup = cg
    while cg != None:
      cgLast = cg
      cg = cg.getGroup

    if cgLast != None:
      # Command was part of CommandGroup, cancel the entire group
      cgLast.cancel()
    else:
      # Stand along command, can call cancel directly
      self.cancel()

  def hasFrontFullyExtended(self):
    """
    Will return true if the front leg extended sensor has been 
    tripped since the command started.
    """
    return self.frontLeg.hasExtended(self.frontLegExtended)

  def hasFrontFullyRetracted(self):
    """
    Will return true if the front leg retracted sensor has been 
    tripped since the command started.
    """
    return self.frontLeg.hasRetracted(self.frontLegRetracted)

  def hasBackFullyExtended(self):
    """
    Will return true if the back leg extended sensor has been 
    tripped since the command started.
    """
    return self.backLeg.hasExtended(self.backLegExtended)

  def hasBackFullyRetracted(self):
    """
    Will return true if the back leg retracted sensor has been 
    tripped since the command started.
    """
    return self.backLeg.hasRetracted(self.backLegRetracted)

  def getLean(self) -> float:
    """
    Returns degrees that robot is leaning forward (-) or backward (+).
    """
    return subsystems.climber.getLean()

  def isLeaningForward(self, lean: float) -> bool:
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
      if lean < -maxLean:
        # Leaning too far backward! Turn off front motor
        # until back catches up
        backPower = 0
      else:
        # Reduce back motor power a bit to let front catch up
        backPower = self.reducePower(backPower, lean)
    else:
      if lean > maxLean:
        # Leaning too far forward! Turn off front motor
        # until back catches up
        frontPower = 0
      else:
        # Reduce front motor power a bit to let back catch up
        frontPower = self.reducePower(frontPower, lean)

    # Apply computed power
    self.frontLeg.setMotorPower(frontPower)
    self.backLeg.setMotorPower(backPower)

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
      self.backLeg.setMotorPower(self.retractSpeed)
    elif lean > backwardLeanLimit:
      # Leaning too far back, need to extend back legs
      self.backLeg.setMotorPower(self.extendSpeed)
    elif lean > (desiredLean - offLean) and lean < (desiredLean + offLean):
      # Back in good range, turn motors off until something changes a lot
      self.backLeg.setMotorPower(0)


class ExtendBothLegs(LiftCommand):
    """ Command that lifts the robot into the air by extending both legs. """
    def __init__(self):
        super().__init__("ExtendBothLegs")

    def execute(self):
        self.fullyExtendBothLegs()

    def isFinished(self):
        """
        Done after once both legs have triggered the fully extended sensors.

        NOTE: They do not need to trigger the sensors simultaneously!
        """
        return self.hasBackFullyExtended() and self.hasFrontFullyExtended()


class RetractFrontLegs(LiftCommand):
    """ Command that retracts the front legs once front wheels are on top platform. """
    def __init__(self):
        super().__init__("RetractFrontLegs")

    def execute(self):
        # Keep back legs level
        self.maintainBackLegs()

        # Retract front legs
        maxLean = self.maxLeanDown
        lean: float = self.getLean()

        if lean < -maxLean:
          # Leaning too far forward, stop retracting front legs until
          # back catches up
          self.frontLeg.setMotorPower(0)
        else:
          self.frontLeg.setMotorPower(self.retractSpeed)

    def isFinished(self):
        # Safety check to completely stop if front is not over floor
        if not self.frontLeg.isOverFloor():
          self.abort()
        return self.hasFrontFullyRetracted()


class RetractBackLegs(LiftCommand):
    """ Command that retracts the back legs once drive wheels are on top platform. """
    def __init__(self):
        super().__init__("RetractBackLegs")

    def execute(self):
        # Retract both legs (keep them up)
        self.backLeg.setMotorPower(self.retractSpeed)

    def isFinished(self):
        # Safety check to completely stop if front is not over floor
        if not self.backLeg.isOverFloor():
          self.abort()
        
        # Safety check to completely stop if we start tipping back too
        # far (means the robot is not far enough on the podium)
        if self.getLean() > self.maxLeanDown:
          self.abort()

        return self.hasBackFullyRetracted()


class DriveToFrontSensor(LiftCommand):
    """ Command that drives forward until the front sensor is over ground. """
    def __init__(self):
        super().__init__("DriveToFrontSensor")

    def execute(self):
        subsystems.climber.setWheelPower(self.wheelPower)
        # Keep robot level with back legs while driving forward
        self.maintainBackLegs()
        # Should we also be driving main drive wheels?
        # Maybe minimal power so climber wheels don't have to fight
        if self.controlDrive:
          subsystems.drive.setPower(0.15, 0.15)

    def isFinished(self):
        return self.frontLeg.isOverFloor()


class DriveToBackSensor(DriveToFrontSensor):
    """ Command that drives forward until the back sensor is over ground. """
    def __init__(self):
        super().__init__()
        self.setName("DriveToBackSensor")

    def isFinished(self):
        return self.backLeg.isOverFloor()


class ClimbUp(CommandGroup):
    """
    Command that takes robot from facing podium all the way
    so that it is on top the podium.
    """
    def __init__(self):
      super().__init__("AutonClimb")
      self.addSequential(ExtendBothLegs())
      self.addSequential(DriveToFrontSensor())
      self.addSequential(RetractFrontLegs())
      self.addSequential(DriveToBackSensor())
      self.addSequential(RetractBackLegs())
      # TODO: Determine command to drive rest of way
