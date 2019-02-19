import wpilib
from wpilib import SmartDashboard
import subsystems
import oi

class DriveTickTimed(wpilib.command.Command):
    """
    An auton test to check how consistent your drive base is when driven purely
    by time.
    """
    kDefaultPower: float = 0.4

    rampUp: int = 0
    cruise: int = 1
    rampDown: int = 2
    done: int = 3

    # Labels for SmartDashboard controls
    fixedLeftLabel: str = "Fixed Left"
    fixedRightLabel: str = "Fixed Right"
    rampTicksLabel: str = "Ramp Ticks"
    cruiseTicksLabel: str = "Cruise Ticks"

    # Current mode
    mode: int = 3

    # Used to count ticks (executions) in each mode.
    ticks: int = 0

    # How many execute runs to ramp power up/down to cruise speed
    rampTicks: int = 20

    # How many execute() runs to keep power at same value while cruising.
    cruiseTicks: int = 40

    # Gain multipliers to adjust left/right fixed values read from the dashboard
    leftGain: float = 1.0
    rightGain: float = 1.0

    # Maximum power to apply to left and right side
    rightPower: float = 0.4
    leftPower: float = 0.4

    def __init__(self, leftGain: float, rightGain: float):
        """
        Initialize all of the control options and control option UI.
        """
        super().__init__("DriveTickTimed", None, subsystems.drive)
        self.leftGain = leftGain
        self.rightGain = rightGain
        self.leftPower = self.rightPower = self.kDefaultPower
        self.mode = self.done
        self.rampTicks = 20
        self.cruiseTicks = 40

        self.leftPower = oi.OI.initializeNumber(self.fixedLeftLabel, self.leftPower)
        self.rightPower = oi.OI.initializeNumber(self.fixedRightLabel, self.rightPower)
        self.rampTicks = int(oi.OI.initializeNumber(self.rampTicksLabel, self.rampTicks))
        self.cruiseTicks = int(oi.OI.initializeNumber(self.cruiseTicksLabel, self.cruiseTicks))

    def initialize(self) -> None:
        """
        Read in and apply current drive choices.
        """
        self.rampTicks = int(SmartDashboard.getNumber(self.rampTicksLabel, self.rampTicks))
        self.cruiseTicks = int(SmartDashboard.getNumber(self.cruiseTicksLabel, self.cruiseTicks))
        self.leftPower = self.leftGain * SmartDashboard.getNumber(self.fixedLeftLabel, self.kDefaultPower)
        self.rightPower = self.rightGain * SmartDashboard.getNumber(self.fixedRightLabel, self.kDefaultPower)
        self.mode = self.rampUp
        self.ticks = 0

    def isFinished(self) -> bool:
        return self.mode == self.done

    def checkModeChange(self, newMode: int, atTickCnt: int) -> None:
        """
        Changes current mode of execute function when tick count is reached.

        : param newMode : The mode to transition when the tick count is reached.
        : param atTickCnt : The tick count that triggers the mode change.
        """
        if self.ticks >= atTickCnt:
            self.mode = newMode
            self.ticks = 0

    def execute(self) -> None:
        """
        Drive robot according to options specified during initialization.
        """
        self.ticks += 1

        if self.mode == self.rampUp:
            # NOTE: To accelerate quickly we really want to allow the ramp power to go up to
            # a higher value than the cruise power and then change mode once we are near the
            # cruise velocity
            rampPower: float = float(self.ticks) / float(self.rampTicks)
            subsystems.drive.setPower(self.leftPower * rampPower, self.rightPower * rampPower)
            self.checkModeChange(self.cruise, self.rampTicks)
            SmartDashboard.putNumber("Ramp Up Velocity (left)", subsystems.drive.getLeft().getVelocity())
            SmartDashboard.putNumber("Ramp Up Velocity (right)", subsystems.drive.getRight().getVelocity())

        elif self.mode == self.cruise:
            subsystems.drive.setPower(self.leftPower, self.rightPower)
            self.checkModeChange(self.rampDown, self.cruiseTicks)
            # We hope these values remain close to the same
            SmartDashboard.putNumber("Cruise Velocity (left)", subsystems.drive.getLeft().getVelocity())
            SmartDashboard.putNumber("Cruise Velocity (right)", subsystems.drive.getRight().getVelocity())

        elif self.mode == self.rampDown:
            rampPower = float(self.rampTicks - self.ticks) / float(self.rampTicks)
            subsystems.drive.setPower(self.leftPower * rampPower, self.rightPower * rampPower)
            self.checkModeChange(self.done, self.rampTicks)
            # We want this to be at zero
            SmartDashboard.putNumber("Ramp Down Velocity (left)", subsystems.drive.getLeft().getVelocity())
            SmartDashboard.putNumber("Ramp Down Velocity (right)", subsystems.drive.getRight().getVelocity())

        else:
            subsystems.drive.stop()

    def end(self) -> None:
        """ Called once after isFinished returns True. """
        subsystems.drive.stop()

    def interrupted(self) -> None:
        """
        Called when another command which requires one or more of the same
        subsystems is scheduled to run.
        """
        self.end()
