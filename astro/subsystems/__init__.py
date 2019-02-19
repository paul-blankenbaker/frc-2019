# This module intializes all subsystems on the robot and makes them
# available to commands

from .drive import Drive
from .climber import Climber

drive : Drive = None
climber : Climber = None

def initialize():
    global drive
    global climber
    drive = Drive()
    climber = Climber()
