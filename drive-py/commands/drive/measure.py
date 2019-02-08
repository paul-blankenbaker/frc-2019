import subsystems
import wpilib
from wpilib.smartdashboard import SmartDashboard

class Measure(wpilib.command.Command):
    """ Utility command that measures drive distance and rotation (zeros each time it is started). """
    def __init__(self):
        super().__init__('Measure')
        # IMPORTANT. Since we are only reading data and not manipulting mechanisms, we do not need
        # to require the drive subsystem
        self.drive = subsystems.drive
        self.left = self.drive.getLeft()
        self.right = self.drive.getRight()
        self.setRunWhenDisabled(True)

    def initialize(self):
        """ Takes initial values of things measurements to "zero" each time the command is started. """ 
        self.yawLast = self.drive.getAngle()
        self.leftDistLast = self.left.getDistance()
        self.leftCntsLast = self.left.getCounts()
        self.rightDistLast = self.right.getDistance()
        self.rightCntsLast = self.right.getCounts()

    def execute(self):
        yawRaw = self.drive.getAngle()
        yaw = yawRaw - self.yawLast
        leftDist = self.left.getDistance() - self.leftDistLast
        leftCnts = self.left.getCounts() - self.leftCntsLast
        leftVel = self.left.getVelocity()
        rightDist = self.right.getDistance() - self.rightDistLast
        rightCnts = self.right.getCounts() - self.rightCntsLast
        rightVel = self.right.getVelocity()

        SmartDashboard.putNumber("Yaw NavX", yawRaw)
        SmartDashboard.putNumber("Yaw Measured", yaw)
        SmartDashboard.putNumber("Left Dist", leftDist)
        SmartDashboard.putNumber("Left Cnts", leftCnts)
        SmartDashboard.putNumber("Left Vel", leftVel)
        SmartDashboard.putNumber("Right Dist", rightDist)
        SmartDashboard.putNumber("Right Cnts", rightCnts)
        SmartDashboard.putNumber("Right Vel", rightVel)

        SmartDashboard.putNumber("Accel X", subsystems.drive.getAccelX())
        SmartDashboard.putNumber("Accel Y", subsystems.drive.getAccelY())
        SmartDashboard.putBoolean("Bump", subsystems.drive.bumpCheck())

    def isFinished(self):
        return False
