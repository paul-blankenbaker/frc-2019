import ctre
from ctre.basemotorcontroller import BaseMotorController

import wpilib
from wpilib import SmartDashboard
from wpilib.command.subsystem import Subsystem

import subsystems
import robotmap

# Generic name for this subsystem
group: str = "Climber"
# Timeout for CAN commands
timeout: int = 10

def initializeMotorController(mc: BaseMotorController):
  """
  Initializes a motor controller to an "initial state" (applies common settings).

  : param mc : A VictorSPX or TalonSRX to initialize.
  """
  if not wpilib.RobotBase.isSimulation():
    mc.configFactoryDefault()
    mc.configFactoryDefault(timeout)
    mc.clearStickyFaults(timeout)
    mc.setSafetyEnabled(False)
    mc.setNeutralMode(ctre.NeutralMode.Brake)
    mc.setInverted(False)

class Leg(object):
  """ Helper class to manage a single leg and floor sensor (robot has two). """

  # Set to True when debugging or testing
  debug: bool = True

  # Name associated with this specific instance (like "Front")
  name: str

  # Used to treat floor detector like a digital input
  floorSensor: wpilib.AnalogInput
  floorDetector: wpilib.AnalogTrigger = None

  # Sensor to detect when leg is fully retracted
  retractedSensor: wpilib.DigitalInput
  retractedCounter: wpilib.Counter

  # Sensor to detect when leg is fully extended
  extendedSensor: wpilib.DigitalInput
  extendedCounter: wpilib.Counter

  # Motor used to extend and retract leg
  legMotor: BaseMotorController

  def __init__(self, name: str, legMotor: BaseMotorController, extendedId: int, retractedId: int, floorId: int):
    """
    Construct new Leg instance.

    : param name : Generic name for leg (like "Front").
    : param legMotor : The motor controller to use for the leg.
    : param extendedId : DIO port ID for sensor at fully extended position.
    : param retractedId : DIO port ID for sensor at fully retracted position.
    : param floorId : Analog port ID for floor sensor by leg.
    """
    self.name = name

    self.floorSensor = wpilib.AnalogInput(floorId)
    self.floorSensor.setName(group, name + " Floor Sensor")

    if not wpilib.RobotBase.isSimulation():
      self.floorDetector = wpilib.AnalogTrigger(floorId) # (wpilib.AnalogInput(floorId))
      self.floorDetector.setName(group, name + " Floor Detect")
      # If voltage goes below 1.0 change to False, when it goes above 2.0 change to True
      self.floorDetector.setLimitsVoltage(1.0, 2.0)

    initializeMotorController(legMotor)
    legMotor.setName(group, name + " Leg Motor")
    legMotor.setInverted(True)
    legMotor.configContinuousCurrentLimit(20, timeout)
    legMotor.enableCurrentLimit(True)
    legMotor.configVoltageCompSaturation(9, timeout)
    legMotor.enableVoltageCompensation(True)

    self.legMotor = legMotor

    self.retractedSensor = wpilib.DigitalInput(retractedId)
    self.retractedSensor.setName(group, name + " Leg Retracted")
    self.retractedCounter = wpilib.Counter(self.retractedSensor)
    self.retractedCounter.setName(group, name + " Retract Count")

    self.extendedSensor = wpilib.DigitalInput(extendedId)
    self.extendedSensor.setName(group, name + " Leg Extended")
    self.extendedCounter = wpilib.Counter(self.extendedSensor)
    self.extendedCounter.setName(group, name + " Extend Count")

  def isExtended(self) -> bool:
    """
    Determine if leg is position such that the fully extended sensor is tripped.
    : return : State of sensor at the fully extended position.
    """
    return self.extendedSensor.get()

  def getExtendedCount(self) -> int:
    """
    Get the current number of times that the sensor at the extended position has been triggered.
    : return : Count that keeps incrementing as sensor is tripped.
    """
    return self.extendedCounter.get()

  def hasExtended(self, priorCount: int) -> bool:
    """
    Determine if the leg has reached the extended position since a prior point in time.

    IMPORTANT: This is the safest way to determine whether or not you've reached the fully
    extended position as it catches the condition where the leg has moved off the sensor.

    : param priorCount : The number of extensions from a prior call to getExtendedCount().

    : return : True if sensor is tripped OR the number of times the sensor
    has been tripped does not match the count you passed in.
    """
    return self.isExtended() or (self.getExtendedCount() != priorCount)

  def isRetracted(self) -> bool:
    """
    Determine if leg is position such that the fully retracted sensor is tripped.
    : return : State of sensor at the fully retracted position.
    """
    return self.retractedSensor.get()

  def getRetractedCount(self) -> int:
    """
    Get the current number of times that the sensor at the retracted position has been triggered.
    : return : Count that keeps incrementing as sensor is tripped.
    """
    return self.retractedCounter.get()

  def hasRetracted(self, priorCount: int) -> bool:
    """
    Determine if the leg has reached the retracted position since a prior point in time.

    IMPORTANT: This is the safest way to determine whether or not you've reached the fully
    retracted position as it catches the condition where the leg has moved off the sensor.

    : param priorCount : The number of retractions from a prior call to
    getRetractedCount().

    : return : True if sensor is tripped OR the number of times the sensor
    has been tripped does not match the count you passed in.
    """
    return self.isRetracted() or (self.getRetractedCount() != priorCount)

  def isOverFloor(self) -> bool:
    """
    Indicates whether the floor sensor by the leg indicates if
    we are over the floor or up in the air.

    : return : True if over floor, False if up in air.
    """
    if self.floorDetector == None:
      # Simulator does not support AnalogTrigger objects
      return self.floorSensor.getVoltage() > 1.5
    return self.floorDetector.getTriggerState()

  def setMotorPower(self, power: float):
    """
    Set the power on the motor that extends and retracts the leg.

    NOTE: There are NO SAFETY CHECKS. Make sure that your command stops
    the motor when fully extended or retracted.

    : param power : Power to apply in range of [-1, +1]. Positive
    to extend, negative to retract.
    """
    return self.legMotor.set(power)

  def periodic(self):
    """ Periodic checks and dashboard updates. """
    n = self.name
    if self.debug == True:
      SmartDashboard.putBoolean(n + " Extended", self.isExtended())
      SmartDashboard.putBoolean(n + " Retracted", self.isRetracted())
      SmartDashboard.putNumber(n + " Extend Cnt", self.getExtendedCount())
      SmartDashboard.putNumber(n + " Retract Cnt", self.getRetractedCount())
      SmartDashboard.putNumber(n + " On Ground", self.isOverFloor())


class Climber(Subsystem):
  """
  Subsystem used to manage the components used to lift the
  robot and drive it onto the platform at the end of the game.
  """

  debug: bool = True

  # Used to manage front and back legs of robot
  frontLeg: Leg
  backLeg: Leg

  # Used to operate wheels at bottom of back legs
  wheels: BaseMotorController


  def __init__(self):
    super().__init__(group)

    frontLegMotor: ctre.WPI_TalonSRX = ctre.WPI_TalonSRX(robotmap.kCanClimbFrontLeg)
    self.frontLeg = Leg("Front", frontLegMotor, robotmap.kDioClimbFrontTop, robotmap.kDioClimbFrontBot, robotmap.kAiClimbGroundFront)

    backLegMotor: ctre.WPI_TalonSRX = ctre.WPI_TalonSRX(robotmap.kCanClimbBackLeg)
    self.backLeg = Leg("Back", backLegMotor, robotmap.kDioClimbBackTop, robotmap.kDioClimbBackBot, robotmap.kAiClimbGroundBack)

    wheelLeftMotor: ctre.WPI_VictorSPX = ctre.WPI_VictorSPX(robotmap.kCanClimbLeftWheel)
    initializeMotorController(wheelLeftMotor)
    wheelLeftMotor.setName(group, "Left Wheel")

    wheelRightMotor: ctre.WPI_VictorSPX = ctre.WPI_VictorSPX(robotmap.kCanClimbRightWheel)
    initializeMotorController(wheelRightMotor)
    wheelRightMotor.setName(group, "Right Wheel")
    wheelRightMotor.follow(wheelLeftMotor)

    # Operate wheel motors as one unit
    self.wheels = wheelLeftMotor

  def getFrontLeg(self) -> Leg:
    """
    Get access to sensors and motors associated with leg at front of robot.
    : return : Leg object that you can query and control.
    """
    return self.frontLeg

  def getBackLeg(self) -> Leg:
    """
    Get access to sensors and motors associated with leg at back of robot.
    : return : Leg object that you can query and control.
    """
    return self.backLeg

  def getLean(self) -> float:
    """
    Get the number of degrees the robot is tipping forward (-) or backward (+).
    : return : A negative value if leaning forward, positive if leaning backward.
    """
    return subsystems.drive.getRoll()

  def setWheelPower(self, power: float = 0.25):
    """
    Sets power on wheels attached to back legs.
    : param power : Power value to apply in range of [-1.0, +1.0].
    """
    self.wheels.set(power)

  def stopWheels(self):
    """ Stops back leg wheels from spinning. """
    self.wheels.set(0)

  def stopAll(self):
    """ Stops all leg and wheel motors. """
    self.frontLeg.setMotorPower(0)
    self.backLeg.setMotorPower(0)
    self.stopWheels()

  def periodic(self):
    """ Periodic checks, sensor readings and dashboard updates. """
    self.frontLeg.periodic()
    self.backLeg.periodic()

