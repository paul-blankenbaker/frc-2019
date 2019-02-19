import ctre
import wpilib
from wpilib import SmartDashboard
from wpilib.command.subsystem import Subsystem

from subsystems import drive
import robotmap

class Climber(Subsystem):
  """
  Subsystem used to manage the components used to lift the
  robot and drive it onto the platform at the end of the game.
  """

  def __init__(self):
    super().__init__('Climber')
    self.debug: bool = True
    timeout = 10
    group = "Climber"

    self.frontFloorSensor: wpilib.AnalogInput = wpilib.AnalogInput(robotmap.kAiClimbGroundFront)
    self.frontFloorSensor.setName(group, "Front Floor Detect")

    self.backFloorSensor: wpilib.AnalogInput = wpilib.AnalogInput(robotmap.kAiClimbGroundBack)
    self.backFloorSensor.setName(group, "Back Floor Detect")

    self.backLiftMotor: ctre.WPI_TalonSRX = ctre.WPI_TalonSRX(robotmap.kCanClimbBackLeg)
    self.backLiftMotor.setName(group, "Back Leg Motor")
    self.backLiftMotor.setInverted(True)

    self.frontLiftMotor: ctre.WPI_TalonSRX = ctre.WPI_TalonSRX(robotmap.kCanClimbFrontLeg)
    self.frontLiftMotor.setName(group, "Front Leg Motor")
    self.frontLiftMotor.setInverted(True)

    wheelLeftMotor: ctre.WPI_VictorSPX = ctre.WPI_VictorSPX(robotmap.kCanClimbLeftWheel)
    wheelLeftMotor.setName(group, "Left Wheel")

    wheelRightMotor: ctre.WPI_VictorSPX = ctre.WPI_VictorSPX(robotmap.kCanClimbRightWheel)
    wheelRightMotor.setName(group, "Right Wheel")

    self.frontRetractedSensor: wpilib.DigitalInput = wpilib.DigitalInput(robotmap.kDioClimbFrontBot)
    self.frontRetractedSensor.setName(group, "Front Leg Retracted")

    self.frontExtendedSensor: wpilib.DigitalInput = wpilib.DigitalInput(robotmap.kDioClimbFrontTop)
    self.frontExtendedSensor.setName(group, "Front Leg Extended")

    self.backRetractedSensor: wpilib.DigitalInput = wpilib.DigitalInput(robotmap.kDioClimbBackBot)
    self.backRetractedSensor.setName(group, "Back Leg Retracted")

    self.backExtendedSensor: wpilib.DigitalInput = wpilib.DigitalInput(robotmap.kDioClimbBackTop)
    self.backExtendedSensor.setName(group, "Back Leg Extended")

    # Apply common motor settings
    for motor in [self.backLiftMotor, self.frontLiftMotor, wheelLeftMotor, wheelRightMotor]:
        motor.configFactoryDefault(timeout)
        motor.clearStickyFaults(timeout)
        motor.setSafetyEnabled(False)
        motor.setNeutralMode(ctre.NeutralMode.Brake)
        motor.setInverted(False)

    # Operate wheel motors as one unit
    self.wheels = wheelLeftMotor
    wheelRightMotor.follow(self.wheels)

    for motor in [self.backLiftMotor, self.frontLiftMotor]:
        motor.configContinuousCurrentLimit(
            20, timeout)  # 15 Amps per motor
        motor.enableCurrentLimit(True)
        motor.configVoltageCompSaturation(
            9, timeout)  # Sets saturation value
        # Compensates for lower voltages
        motor.enableVoltageCompensation(True)

  def getLean(self) -> float:
    """
    Get the number of degrees the robot is tipping forward (-) or backward (+).
    : return : A negative value if leaning forward, positive if leaning backward.
    """
    return drive.getRoll()

  def isFullyExtendedFront(self) -> bool:
    """
    Indicates whether front leg is fully extended.

    : return : True when front leg is extended such that sensor is tripped.
    """
    return self.frontExtendedSensor.get()

  def isFullyExtendedBack(self) -> bool:
    """
    Indicates whether back leg is fully extended.

    : return : True when back leg is extended such that sensor is tripped.
    """
    return self.backExtendedSensor.get()

  def isFullyExtendedBoth(self) -> bool:
    """
    Indicates whether both legs are fully extended.

    : return : True when both legs are extended such that both sensors are tripped.
    """
    return self.isFullyExtendedFront() and self.isFullyExtendedBack()

  def isFullyRetractedFront(self) -> bool:
    """
    Indicates whether front leg is fully retracted.

    : return : True when front leg is retracted such that sensor is tripped.
    """
    return self.frontRetractedSensor.get()

  def isFullyRetractedBack(self) -> bool:
    """
    Indicates whether back leg is fully retracted.

    : return : True when back leg is retracted such that sensor is tripped.
    """
    return self.backRetractedSensor.get()

  def isFullyRetractedBoth(self) -> bool:
    """
    Indicates whether both legs are fully retracted.

    : return : True when both legs are retracted such that both sensors are tripped.
    """
    return self.isFullyRetractedFront() and self.isFullyRetractedBack()

  def isFrontOverGround(self) -> bool:
    """
    Checks to see if the front floor sensor is detecting the ground.
    : return : True if sensor is tripped indicating that it is over ground.
    """
    return self.frontFloorSensor.getVoltage() < 1.5

  def isBackOverGround(self):
    """
    Checks to see if the back floor sensor is detecting the ground.
    : return : True if sensor is tripped indicating that it is over ground.
    """
    return self.backFloorSensor.getVoltage() < 1.5

  def moveFrontLegs(self, lift: float, maxLean: float = 2.0):
    """
    Control power output of back leg lift motor to extend/retract leg.
  
    :param lift : Power for leg motor - positive to extend, negative to retract.
    :param maxLean : Maximum lean allowed before stopping in degrees.
    """

    lean = self.getLean()

    if lift > 0:
        # They want to extend legs (robot go up)
        if self.isFullyExtendedFront():
            self.stopFront()
        elif lean < maxLean:
            # Not leaning too far back, OK to extend front legs
            self.frontLiftMotor.set(lift)
        else:
            self.stopFront()
    else:
        # They want legs to retract (robot go down)
        if self.isFullyRetractedFront():
            self.stopFront()
        elif lean > -maxLean:
            # Not leaning too far forward, OK to retract front legs
            self.frontLiftMotor.set(lift)
        else:
            self.stopFront()

  def moveBackLegs(self, lift: float, maxLean: float = 2.0):
    """
    Control power output of front leg lift motor to extend/retract leg.
  
    :param lift : Power for leg motor - positive to extend, negative to retract.
    :param maxLean : Maximum lean allowed before stopping in degrees.
    """
    lean = self.getLean()

    if lift > 0:
        # They want to extend legs (robot go up)
        if self.isFullyExtendedBack():
            self.stopBack()
        elif lean > -maxLean:
            # Not leaning too far forward, OK to extend back legs
            self.backLiftMotor.set(lift)
        else:
            self.stopBack()
    else:
        # They want legs to retract (robot go down)
        if self.isFullyRetractedBack():
            self.stopBack()
        elif lean < maxLean:
            # Not leaning too far backward, OK to retract back legs
            self.backLiftMotor.set(lift)
        else:
            self.stopBack()

  def setWheelPower(self, power: float = 0.25):
    """
    Sets power on wheels attached to back legs.
    : param power : Power value to apply in range of [-1.0, +1.0].
    """
    self.wheels.set(power)

  def stopFront(self):
    """ Stops front leg motor. """
    self.frontLiftMotor.set(0)

  def stopBack(self):
    """ Stops back leg motor. """
    self.backLiftMotor.set(0)

  def stopDrive(self):
    """ Stops back leg wheels from spinning. """
    self.wheels.set(0)

  def stopAll(self):
    """ Stops all leg and wheel motors. """
    self.stopFront()
    self.stopBack()
    self.stopDrive()

  def periodic(self):
    """ Periodic checks and dashboard updates. """
    if self.debug == True:
      SmartDashboard.putBoolean("Front Extended", self.isFullyExtendedFront())
      SmartDashboard.putBoolean("Back Extened", self.isFullyExtendedBack())
      SmartDashboard.putBoolean("Front Retracted", self.isFullyRetractedFront())
      SmartDashboard.putBoolean("Back Retracted", self.isFullyRetractedBack())
      SmartDashboard.putBoolean("Robot Lean", self.getLean())
      SmartDashboard.putNumber("Front On Ground", self.isFrontOverGround())
      SmartDashboard.putNumber("Back On Ground", self.isBackOverGround())

