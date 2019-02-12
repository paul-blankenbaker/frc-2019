import subsystems
import oi

import wpilib
from wpilib.command import Command
from wpilib.drive.differentialdrive import DifferentialDrive
from wpilib.sendablechooser import SendableChooser
from wpilib.smartdashboard import SmartDashboard

from commands.drive.measure import Measure

# Dashboard control to select drive mode
modeChooser : SendableChooser = None

# Used to indicate which end of the robot is the front
isFlipped : bool = False

# Used to control whether brake mode is enabled on the motor controllers
enableBrakeMode : bool = False

class FlipFront(wpilib.command.InstantCommand):

    def __init__(self):
        super().__init__("FlipFront")
        self.updateDashboard()

    def initialize(self):
        global isFlipped
        isFlipped = not isFlipped
        self.updateDashboard()

    def updateDashboard(self):
        global isFlipped
        SmartDashboard.putBoolean("Flipped Front", isFlipped)

class ToggleBrakeMode(wpilib.command.InstantCommand):

    def __init__(self):
        super().__init__("ToggleBrakeMode")
        self.updateDashboard()

    def initialize(self):
        global enableBrakeMode
        enableBrakeMode = not enableBrakeMode
        self.updateDashboard()

    def updateDashboard(self):
        global enableBrakeMode
        SmartDashboard.putBoolean("Brake Mode", enableBrakeMode)


# Drive mode choices
kModeArcade : int = 0
kModeTank : int = 1
kModeCurvature : int = 2
kModeFixed : int = 3

class DriveHuman(Command):

    kThrottleAxis : int = 1
    kRotationAxis : int = 4
    kLeftAxis : int = 1
    kRightAxis : int = 3

    def __init__(self):
        """ Initialize all of the control options and control option UI. """
        super().__init__("HumanDrive", None, subsystems.drive)
        self.squareInputs : bool = True
        self.quickTurn : bool = False
        self.mode : int = kModeArcade
        self.fixedLeft : float = 0.4
        self.fixedRight : float = 0.4
        self.rotationGain : float = 0.5
        self.slowGain : float = 0.5
        self.minDeflect : float = 1.0 / 32.0
        self.dd: DifferentialDrive = subsystems.drive.getDifferentialDrive()

    @staticmethod
    def createFlipFrontCommand() -> Command:
        return FlipFront()

    @staticmethod
    def createBrakeModeToggleCommand() -> Command:
        return ToggleBrakeMode()

    @staticmethod
    def setupDashboardControls():
        # Only need to set up dashbard drive controls once
        global modeChooser
        if modeChooser == None:
            SmartDashboard.putBoolean("Quick Turn", False)
            SmartDashboard.putBoolean("Square Inputs", True)
            SmartDashboard.putNumber("Fixed Left", 0.4)
            SmartDashboard.putNumber("Fixed Right", 0.4)
            SmartDashboard.putNumber("Rotation Gain", 0.5)
            SmartDashboard.putNumber("Slow Gain", 0.5)

            mc = SendableChooser()
            mc.addDefault("Arcade", kModeArcade)
            mc.addOption("Tank", kModeTank)
            mc.addOption("Curvature", kModeCurvature)
            mc.addOption("Fixed", kModeFixed)
            SmartDashboard.putData("Drive Mode", mc)
            modeChooser = mc

    def initialize(self):
        """ Read in and apply current drive choices. """
        DriveHuman.setupDashboardControls()
        global modeChooser
        self.mode = modeChooser.getSelected()
        self.quickTurn = SmartDashboard.getBoolean("Quick Turn", self.quickTurn)
        self.squareInputs = SmartDashboard.getBoolean("Square Inputs", self.squareInputs)
        self.fixedLeft = SmartDashboard.getNumber("Fixed Left", self.fixedLeft)
        self.fixedRight = SmartDashboard.getNumber("Fixed Right", self.fixedRight)
        self.rotationGain = SmartDashboard.getNumber("Rotation Gain", self.rotationGain)
        self.slowGain = SmartDashboard.getNumber("Slow Gain", self.slowGain)
    
    def execute(self):
        """ Drive robot according to options specified during initialization. """
        global isFlipped
        gain : float = 1.0
        rotGain = self.rotationGain

        # Slow-mo mode when user is holding down on button 6
        if oi.instance.readDriverButton(6):
            gain = self.slowGain
            rotGain = self.slowGain

        if self.mode == kModeArcade:
            throttle = -oi.instance.readDriverAxis(self.kThrottleAxis) * gain
            rotation = oi.instance.readDriverAxis(self.kRotationAxis) * rotGain
            if isFlipped:
                throttle = -throttle
            self.dd.arcadeDrive(throttle, rotation, self.squareInputs)
        elif self.mode == kModeTank:
            left = -oi.instance.readDriverAxis(self.kLeftAxis) * gain
            right = -oi.instance.readDriverAxis(self.kRightAxis) * gain
            if isFlipped:
                tmpLeft = -left
                left = -right
                right = tmpLeft
            self.dd.tankDrive(left, right, self.squareInputs)
        elif self.mode == kModeCurvature:
            throttle = -oi.instance.readDriverAxis(self.kThrottleAxis) * gain
            rotation = oi.instance.readDriverAxis(self.kRotationAxis) * rotGain
            if isFlipped:
                throttle = -throttle
            self.dd.curvatureDrive(throttle, rotation, self.quickTurn)
        elif self.mode == kModeFixed:
            self.driveFixed(gain, rotGain)
        else:
            # Unknown mode, stop driving
            subsystems.drive.stop()

    def isFinished(self) -> bool:
        return False

    def end(self):
        subsystems.drive.stop()

    def interrupted(self):
        self.end()

    def driveFixed(self, gain : float, rotGain : float):
        """ Drives left and right side motors at fixed power value specified on the dashboard. """
        global isFlipped

        leftPower = 0
        rightPower = 0
        throttle = -oi.instance.readDriverAxis(self.kThrottleAxis)
        rotation = oi.instance.readDriverAxis(self.kRotationAxis)
        if isFlipped:
            throttle = -throttle

        if throttle > self.minDeflect:
            leftPower = self.fixedLeft
            rightPower = self.fixedRight
        elif throttle < -self.minDeflect:
            leftPower = -self.fixedLeft
            rightPower = -self.fixedRight
        else:
            # No forward/reverse throttle, check for rotation
            gain = rotGain
            if rotation > self.minDeflect:
                leftPower = self.fixedLeft
                rightPower = -self.fixedRight
            elif rotation < -self.minDeflect:
                leftPower = -self.fixedLeft
                rightPower = self.fixedRight
        # Apply determined power value
        subsystems.drive.setPower(leftPower * gain, rightPower * gain)


