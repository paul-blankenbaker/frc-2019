from wpilib import Preferences

# Robot preferences file stored on roboRIO
# values can be set differently for each roboRIO
config: Preferences = Preferences.getInstance()

def getConfigInt(key: str, defVal: int) -> int:
  """
  Looks up value an integer value from the robot configuration file
  or creates the value if not present.

  : param key : Key to use to look up/set value.
  : param defVal : Default value to set/return if not found.
  : return : Value from configuration file or default if not found.
  """
  global config
  if config.containsKey(key):
    val: int = config.getInt(key, defVal)
  else:
    # Value not set in config, set to default value provided
    # so we will see it and be able to edit it in the system
    # preferences editor
    val: int = defVal
    config.putInt(key, val)
  return val

# Known robots that might have slight variations in configuration
# that we want to deploy the code to
kAstroV2: int = 0
kAstroV1: int = 1
kSynapse: int = 2

# ID of robot we are deploying to
kRobotId: int = getConfigInt("RobotId", kAstroV2)

# Will be True if deploying to competition robot
kCompetitionBot: bool = (kRobotId == kAstroV2)

# Map of CAN ID allocation
kCanDriveLeft0: int = 10
kCanDriveLeft1: int = 11
kCanDriveLeft2: int = 12

kCanDriveRight0: int = 20
kCanDriveRight1: int = 21
kCanDriveRight2: int = 22

kCanClimbFrontLeg: int = 40
kCanClimbBackLeg: int = 41
kCanClimbLeftWheel: int = 50
kCanClimbRightWheel: int = 51

# Map of DIO allocation
kDioDriveLeftEncA: int  = 0
kDioDriveLeftEncB: int = 1
kDioDriveRightEncA: int = 2
kDioDriveRightEncB: int = 3

kDioClimbFrontTop: int = 7
kDioClimbFrontBot: int = 8
kDioClimbBackTop: int = 9
kDioClimbBackBot: int = 6

# Map of Analog Input allocation
kAiClimbGroundFront: int = 0
kAiClimbGroundBack: int = 1
