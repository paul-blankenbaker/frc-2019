import subsystems
import oi
import wpilib
from wpilib.smartdashboard import SmartDashboard
from wpilib.sendablechooser import SendableChooser
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



class DriveHuman(wpilib.command.Command):
    # Drive mode choices
    kModeArcade : int = 0
    kModeTank : int = 1
    kModeCurvature : int = 2
    kModeFixed : int = 3

    kThrottleAxis : int = 1
    kRotationAxis : int = 4
    kLeftAxis : int = 1
    kRightAxis : int = 3

    def __init__(self):
        """ Initialize all of the control options and control option UI. """
        super().__init__("HumanDrive", None, subsystems.drive)
        global modeChooser
        self.squareInputs : bool = True
        self.quickTurn : bool = False
        self.mode : int = self.kModeArcade
        self.fixedLeft : float = 0.4
        self.fixedRight : float = 0.4
        self.rotationGain : float = 0.5
        self.slowGain : float = 0.5
        self.minDeflect : float = 1 / 64

        # Only need to set up dashbard drive controls once
        if modeChooser == None:
            SmartDashboard.putBoolean("Quick Turn", self.quickTurn)
            SmartDashboard.putBoolean("Square Inputs", self.squareInputs)
            SmartDashboard.putNumber("Fixed Left", self.fixedLeft)
            SmartDashboard.putNumber("Fixed Right", self.fixedRight)
            SmartDashboard.putNumber("Rotation Gain", self.rotationGain)
            SmartDashboard.putNumber("Slow Gain", self.slowGain)

            mc = SendableChooser()
            mc.addDefault("Arcade", self.kModeArcade)
            mc.addOption("Tank", self.kModeTank)
            mc.addOption("Curvature", self.kModeCurvature)
            mc.addOption("Fixed", self.kModeFixed)
            SmartDashboard.putData("Drive Mode", mc)
            modeChooser = mc

            SmartDashboard.putData("Brake Control", ToggleBrakeMode())
            SmartDashboard.putData("Flip Front", FlipFront())
            flipButton = oi.instance.createDriverButton(5)
            flipButton.whenPressed(FlipFront())

    def initialize(self):
        """ Read in and apply current drive choices. """
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

        if self.mode == self.kModeArcade:
            throttle = -oi.instance.readDriverAxis(self.kThrottleAxis) * gain
            rotation = oi.instance.readDriverAxis(self.kRotationAxis) * rotGain
            if isFlipped:
                throttle = -throttle
            subsystems.drive.arcadeDrive(throttle, rotation, self.squareInputs)
        elif self.mode == self.kModeTank:
            left = -oi.instance.readDriverAxis(self.kLeftAxis) * gain
            right = -oi.instance.readDriverAxis(self.kRightAxis) * gain
            if isFlipped:
                tmpLeft = -left
                left = -right
                right = tmpLeft
            subsystems.drive.tankDrive(left, right, self.squareInputs)
        elif self.mode == self.kModeCurvature:
            throttle = -oi.instance.readDriverAxis(self.kThrottleAxis) * gain
            rotation = oi.instance.readDriverAxis(self.kRotationAxis) * rotGain
            if isFlipped:
                throttle = -throttle
            subsystems.drive.curvatureDrive(throttle, rotation, self.quickTurn)
        elif self.mode == self.kModeFixed:
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


