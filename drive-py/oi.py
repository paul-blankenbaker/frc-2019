import wpilib
from wpilib import SmartDashboard

from wpilib.joystick import Joystick
from wpilib.buttons.joystickbutton import JoystickButton

import subsystems

from commands.drive.measure import Measure

class OI():

    def __init__(self):
        self.driver: Joystick = Joystick(0)
        self.setupDriverCommands()
        self.setupDashboardCommands()

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
        pass

    def setupDashboardCommands(self):
        SmartDashboard.putData("Measure", Measure())

""" Single instance of OI object """
instance : OI = None

def initialize():
    global instance
    instance = OI()
