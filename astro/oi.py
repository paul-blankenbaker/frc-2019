import wpilib
from wpilib import SmartDashboard
from wpilib.command import Command
from wpilib.sendablechooser import SendableChooser
from wpilib.joystick import Joystick
from wpilib.buttons.joystickbutton import JoystickButton

import subsystems

from commands.drive.measure import Measure
from commands.drive.drivehuman import DriveHuman
from commands.drive.driveticktimed import DriveTickTimed
from commands.loadtest import LoadTest
from commands.lift import ClimbUp, DriveToBackSensor, DriveToFrontSensor, ExtendBothLegs, RetractFrontLegs, RetractBackLegs

class OI():
    debug: bool = True
    autonChooser: SendableChooser = None
    driver: Joystick = None

    def __init__(self):
        self.driver: Joystick = Joystick(0)
        self.setupDriverCommands()
        self.setupDashboardCommands()

    @staticmethod
    def initializeNumber(label: str, defaultValue: float) -> float:
        value: float = SmartDashboard.getNumber(label, defaultValue)
        SmartDashboard.putNumber(label, value)
        return value

    @staticmethod
    def initializeBoolean(label: str, defaultValue: bool) -> bool:
        value: bool = SmartDashboard.getBoolean(label, defaultValue)
        SmartDashboard.putBoolean(label, value)
        return value

    def createDriverButton(self, rawId : int) -> JoystickButton:
        """ Returns a button object that you can attach event handlers (commands)
        to execute when the driver presses, holds or releases the button on the gamepad. """
        return JoystickButton(self.driver, rawId)

    def readDriverAxis(self, rawAxisId : int) -> float:
        """ Reads value of an analog axis on the driver game pad [-1.0, +1.0]. """
        return self.driver.getRawAxis(rawAxisId)

    def readDriverButton(self, rawButtonId : int) -> bool:
        """ Returns true if driver is currently pressing the button specified. """
        return self.driver.getRawButton(rawButtonId)

    def setupDriverCommands(self):
        flipButton = self.createDriverButton(5)
        flipButton.whenPressed(DriveHuman.createFlipFrontCommand())

    def getSelectedAuton(self) -> Command:
        return self.autonChooser.getSelected()

    def setupDashboardCommands(self):
        # Set up auton chooser
        self.autonChooser = SendableChooser()
        self.autonChooser.setDefaultOption("Do Nothing", wpilib.command.WaitCommand(3.0, "Do Nothing"))
        self.autonChooser.addOption("Drive Forward", DriveTickTimed(1.0, 1.0))
        self.autonChooser.addOption("Drive Backward", DriveTickTimed(-1.0, -1.0))
        self.autonChooser.addOption("Rotate Right", DriveTickTimed(1.0, -1.0))
        self.autonChooser.addOption("Rotate Left", DriveTickTimed(-1.0, 1.0))
        SmartDashboard.putData("Auto mode", self.autonChooser)

        # Drive controls
        DriveHuman.setupDashboardControls()
        SmartDashboard.putData("Brake Control", DriveHuman.createBrakeModeToggleCommand())
        SmartDashboard.putData("Flip Front", DriveHuman.createFlipFrontCommand())

        # Set up utility controls
        SmartDashboard.putData("Measure", Measure())

        # Climber settings
        SmartDashboard.putData(ClimbUp())        
        if self.debug:
          SmartDashboard.putData(ExtendBothLegs())
          SmartDashboard.putData(DriveToFrontSensor())
          SmartDashboard.putData(RetractFrontLegs())
          SmartDashboard.putData(DriveToBackSensor())
          SmartDashboard.putData(RetractBackLegs())

        # Debug tools (if enabled)
        if self.debug:
            SmartDashboard.putData("CPU Load Test", LoadTest())
            SmartDashboard.putData("Drive Subsystem", subsystems.drive)
            dd = subsystems.drive.getDifferentialDrive()
            if dd != None:
                SmartDashboard.putData("DifferentialDrive", dd)

""" Single instance of OI object """
instance : OI = None

def initialize():
    global instance
    instance = OI()
