# This module intializes all subsystems on the robot and makes them
# available to commands

from .drive import Drive

drive : Drive = None

def initialize():
    global drive
    drive = Drive()
