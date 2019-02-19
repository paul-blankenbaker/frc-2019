import math
import ctre
import navx
import wpilib
from wpilib import SmartDashboard
from wpilib.command.subsystem import Subsystem
from wpilib.drive.differentialdrive import DifferentialDrive
import robotmap
from commands.drive.drivehuman import DriveHuman 

kTimeout : int = 0

# Converts encoder counts to ticks
kEncCntsPerRev = 256
kWheelDiameter = 5.75
kLeftConv = (kWheelDiameter / 12) * (math.pi / kEncCntsPerRev) 
kRightConv = (kWheelDiameter / 12) * (math.pi / kEncCntsPerRev)

class Tread():
    """ Helper class that provides common operations for one side of drive train. """

    encoder : wpilib.Encoder = None
    # Counts last read from encoder
    encCnts : int = 0
    # Distance last read from encoder
    encDist : float = 0
    # Velocity last returned from encoder
    encVel : float = 0

    # Main motor controller for motor cluster
    motor: ctre.WPI_TalonSRX = None
    # List of motor followers
    followers = ()

    def setPower(self, power: float):
        """ Sets percentage of voltage to allow to motors (control power output).
        : param power : Power output from [-1.0, +1.0] - negative moves robot backward, positive moves robot forward.
        """
        self.motor.set(power)

    def getDistance(self) -> float:
        """ Returns distance traveled in feet. """
        return self.encDist

    def getCounts(self) -> int:
        """ Returns counts last read from encoder. """
        return self.encCnts

    def getVelocity(self) -> float:
        """ Returns velocity last read in ft/sec. """
        return self.encVel

    def zero(self):
        """ Resets the encoder - best to avoid this - may remove eventually. """
        self.encoder.reset()

    def __setVoltageCompensation__(self, voltage : float) -> None:
        self.motor.configVoltageCompSaturation(voltage, kTimeout)
        self.motor.enableVoltageCompensation(True)
        # NOT sure, but I don't think we need to apply this configuration to followers

    def __setBrakeMode__(self, enable : bool) -> None:
        mode = ctre.NeutralMode.Coast
        if enable:
            mode = ctre.NeutralMode.Brake 
            self.motor.setNeutralMode(mode)
            for f in self.followers:
                # Apply configuration to followers as well (not sure if this is required)
                f.setNeutralMode(mode)
            

    @staticmethod
    def createTalonSRX(canId : int, invert : bool) -> ctre.WPI_TalonSRX:
        """ Helper method to create and initialize a TalonSRX speed controller.
        
        : param canId : ID on CAN bus of TalonSRX
        : param invert : Pass true if motor output needs to be inverted """
        s = ctre.WPI_TalonSRX(canId)
        s.clearStickyFaults(kTimeout)
        s.setSafetyEnabled(False)
        s.setInverted(invert)
        s.configContinuousCurrentLimit(15, kTimeout) #15 Amps per motor
        s.configPeakCurrentLimit(20, kTimeout) #20 Amps during Peak Duration
        s.configPeakCurrentDuration(100, kTimeout) #Peak Current for max 100 ms
        s.enableCurrentLimit(True)
        s.configOpenLoopRamp(0.2, kTimeout) #number of seconds from 0 to 1
        return s

    @staticmethod
    def createVictorSPX(leader : ctre.WPI_TalonSRX, canId : int, invert : bool) -> ctre.WPI_VictorSPX:
        """ Helper method to create and initialize a VictorSPX speed controller.
        
        : param leader : The TalonSRX that the VictorSPX should follow
        : param canId : ID on CAN bus of VictorSPX
        : param invert : Pass true if motor output needs to be inverted """
        s = ctre.WPI_VictorSPX(canId)
        s.clearStickyFaults(kTimeout)
        s.setSafetyEnabled(False)
        s.setInverted(invert)
        s.follow(leader)
        return s

    def __init__(self, tname: str, t0: int, v1: int, v2: int, invert: bool, chA: int, chB: int, cntsToFt: float):
        self.debug = False
        self.name = tname
        self.motor = Tread.createTalonSRX(t0, invert)
        self.motor.setName("Drive", tname + "Motor0")
        if not wpilib.RobotBase.isSimulation():
            s1 = Tread.createVictorSPX(self.motor, v1, invert)
            s1.setName("Drive", tname + "Motor1")
            s2 = Tread.createVictorSPX(self.motor, v2, invert)
            s2.setName("Drive", tname + "Motor2")
            self.followers = ( s1, s2 )
        else:
            self.followers = ()

        self.encoder = wpilib.Encoder(chA, chB)
        self.encoder.setName("Drive", tname + "Enc")
        self.encoder.setDistancePerPulse(cntsToFt)
        self.encoder.setSamplesToAverage(10)
        self.__setVoltageCompensation__(12) # Default to 12 volts max to motors
        self.__readSensors__()

    def __readSensors__(self):
        self.encCnts = self.encoder.get()
        self.encDist = self.encoder.getDistance()
        self.encVel = self.encoder.getRate()

    def __dashboard_periodic__(self):
        if self.debug:
            current = self.motor.getOutputCurrent()
            volts = self.motor.getMotorOutputVoltage()
            watts = volts * current
            SmartDashboard.putNumber(self.name + " Talon Current", current)
            SmartDashboard.putNumber(self.name + " Talon Volts", volts)
            SmartDashboard.putNumber(self.name + " Talon Watts", watts)

class Drive(Subsystem):

    def __init__(self):
        super().__init__('Drive')

        self.debug = False
        self.yawZero = 0
        self.rollZero = 0

        self.navx = navx.ahrs.AHRS.create_spi()
        self.navx.setName("Drive", "NavX")
        self.accel = wpilib.BuiltInAccelerometer()
        self.accel.setName("Drive", "Accel")
        self.left : Tread = Tread("Left", \
            robotmap.kCanDriveLeft0, robotmap.kCanDriveLeft1, robotmap.kCanDriveLeft2, True, \
            robotmap.kDioDriveLeftEncA, robotmap.kDioDriveLeftEncB, kLeftConv)
        self.right : Tread = Tread("Right", \
            robotmap.kCanDriveRight0, robotmap.kCanDriveRight1, robotmap.kCanDriveRight2, False, \
            robotmap.kDioDriveRightEncA, robotmap.kDioDriveRightEncB, kRightConv)

        self.left.debug = self.debug
        self.right.debug = self.debug

        self.drive : DifferentialDrive = DifferentialDrive(self.left.motor, self.right.motor)
        self.drive.setName("Drive", "Differential")
        self.drive.setSafetyEnabled(False)
        self.drive.setDeadband(0.025)
        self.drive.setRightSideInverted(False)
        self.periodic()
        self.zero()

    def initDefaultCommand(self):
        self.setDefaultCommand(DriveHuman())

    def setVoltageCompensation(self, maxVoltsLeft: float, maxVoltsRight: float) -> None:
        """
        Sets the voltage range that [-1.0, +1.0] maps to.
     
        For consistent auton, you probably want to set this value to just under the
        lowest value you see your battery voltage drop to when the robot is running.

        You will normally specify the same value for both sides, however if your
        robot pulls hard to one side or the other you can drop the limit to the more
        powerful side to apply a linear correction gain to get it closer to straight.

        : @param maxVoltsLeft : Maximum voltage to use for the left side of the drive
        train when -1 or +1 is specified as power level (typically 12.0 or less).

        : @param maxVoltsRight : Maximum voltage to use for the right side of the drive
        train when -1 or +1 is specified as power level (typically 12.0 or less).
        """
        self.left.__setVoltageCompensation__(maxVoltsLeft)
        self.right.__setVoltageCompensation__(maxVoltsRight)

    def setBrakeMode(self, enable: bool) -> None:
        """
        Controls whether or not the motor controllers will brake or coast when power
        is set to 0.0.
      
        : @param enable : Pass true to enable brake mode, false to enable coast mode.
        """
        self.left.__setBrakeMode__(enable)
        self.right.__setBrakeMode__(enable)
    
    def setPower(self, left: float = 0, right: float = 0):
        """ Use in autonomous commands to directly apply power to the left an right motors. """
        self.left.setPower(left)
        self.right.setPower(right)

    def getDifferentialDrive(self) -> DifferentialDrive:
        return self.drive

    # def arcadeDrive(self, throttle : float, rotation : float, squareInputs : bool = True):
    #     self.drive.arcadeDrive(throttle, rotation, squareInputs)

    # def curvatureDrive(self, throttle : float, rotation : float, isQuickTurn : bool = False):
    #     self.drive.curvatureDrive(throttle, rotation, isQuickTurn)

    # def tankDrive(self, leftPower : float, rightPower : float, squareInputs : bool = True):
    #     self.drive.tankDrive(leftPower, rightPower, squareInputs)

    def periodic(self):
        """
        This method is called automatically by the Scheduler and we take our
        sensor readings once per run cycle here.
        """
        self.__readSensors__()
        self.left.__readSensors__()
        self.right.__readSensors__()
        self.dashboardPeriodic()

    def __readSensors__(self):
        self.yaw = self.navx.getYaw()
        self.roll = self.navx.getRoll()
        self.accelX = self.accel.getX()
        self.accelY = self.accel.getY()
        self.accelZ = self.accel.getZ()

    def getLeft(self) -> Tread:
        """ Use this method if you need specific data or control of the left side of the drive. """
        return self.left

    def getRight(self) -> Tread:
        """ Use this method if you need specific data or control of the right side of the drive. """
        return self.right

    def getAngle(self) -> float:
        """ Returns angle of rotation since last zeroing. """
        return self.yaw - self.yawZero

    def getAngleRaw(self) -> float:
        """ Returns raw angle from NavX (never zeroed). """
        return self.yaw

    def getRoll(self) -> float:
        """
        Returns roll angle sinze last zeroing.
        : return : Degrees leaning forward or backward. Will be negative if leaning forward and positive if leaning backward.
        """
        return self.roll - self.rollZero

    def getRollRaw(self) -> float:
        """
        Returns raw roll value from NavX (never zeroed).
        """
        return self.roll

    def getAccelX(self) -> float:
        """ Returns acceleration in X-axis reported by built in accelerameter. """
        return self.accelX

    def getAccelY(self) -> float:
        """ Returns acceleration in Y-axis reported by built in accelerameter. """
        return self.accelY

    def getAvgDistance(self):
        """ Returns average of distance of left and right side encoders. """
        return (self.left.getDistance() + self.right.getDistance()) / 2

    def getAvgVelocity(self):
        """
        Returns average velocity reported by left/right side encoders.
        Will be zero if spinning or stopped. Will be positive if moving forwards.
        Will be negative if moving backwards.
        """
        return (self.left.getVelocity() + self.right.getVelocity()) / 2

    def getAvgAbsVelocity(self):
        """
        Returns average of velocity magnitudes. Will be zero or close to zero
        if not much movement. Large positive value means robot is spinning,
        moving forward or backward quickly.
        """
        return (abs(self.left.getVelocity()) + abs(self.right.getVelocity())) / 2

    def zeroDistance(self):
        """ Zeros out tracked encoder values. """
        self.left.zero()
        self.right.zero()

    def zeroAngle(self):
        """ Zeros out tracked angle information. """
        self.yawZero = self.yaw
        self.rollZero = self.roll

    def zero(self):
        """ Zeros out tracked distance and angle information. """
        self.zeroDistance()
        self.zeroAngle()

    def stop(self):
        """ Stops all drive motors. """
        self.setPower(0, 0)

    def dashboardPeriodic(self):
        pass

    def bumpCheck(self, bumpX: float = 0.4, bumpY: float = 0.4) -> bool:
        ''' Returns true if magnitude of acceleration from built in 
        accelerometer is greater than bump limit specified.
        
        : param bumpX : Threshold for triggering bump condition in X axis
        : param bumpY : Threshold for triggering bump condition in Y axis
        '''
        xBump = abs(self.getAccelX()) > bumpX
        yBump = abs(self.getAccelY()) > bumpY
        return xBump or yBump
