package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;

/**
 * An auton test to check how consistent your drive base is when driven purely
 * by time.
 */
public class DriveTickTimed extends Command {
  private static final double kDefaultPower = 0.4;

  enum Mode {
    rampUp, cruise, rampDown, done
  };

    // Labels for SmartDashboard controls
    private static final String fixedLeftLabel = DriveHuman.fixedLeftLabel;
    private static final String fixedRightLabel = DriveHuman.fixedRightLabel;
  private static final String rampTicksLabel = "Ramp Ticks";
  private static final String cruiseTicksLabel = "Cruise Ticks";
  

  private Mode mode;

  /** Used to count ticks (executions) in each mode. */
  private int ticks;

  /** How many execute runs to ramp power up/down to cruise speed. */
  private int rampTicks;
  /** How many execute() runs to keep power at same value while cruising. */
  private int cruiseTicks;

  // Gain multipliers to adjust left/right fixed values read from the dashboard
  private final double leftGain;
  private final double rightGain;

  // Maximum power to apply to left and right side
  private double rightPower;
  private double leftPower;
 
  /**
   * Initialize object with multipliers for left/right side of drive train.
   * 
   * @param leftGain - Power multiplier/inverter for left side (typically 1.0 or -1.0).
   * @param rightGain - Power multiplier/inverter for right side (typically 1.0 or -1.0).
   */
  public DriveTickTimed(double leftGain, double rightGain) {
    super("DriveTickTimed", Robot.drive);
    this.leftGain = leftGain;
    this.rightGain = rightGain;
    leftPower = rightPower = kDefaultPower;
    mode = Mode.done;
    rampTicks = 20;
    cruiseTicks = 40;

    leftPower = OI.initializeNumber(fixedLeftLabel, leftPower);
    rightPower = OI.initializeNumber(fixedRightLabel, rightPower);
    rampTicks = (int) OI.initializeNumber(rampTicksLabel, rampTicks);
    cruiseTicks = (int) OI.initializeNumber(cruiseTicksLabel, cruiseTicks);
  }

  /**
   * Read in and apply current drive choices.
   */
  @Override
  public void initialize() {
    rampTicks = (int) SmartDashboard.getNumber(rampTicksLabel, rampTicks);
    cruiseTicks = (int) SmartDashboard.getNumber(cruiseTicksLabel, cruiseTicks);
    leftPower = this.leftGain * SmartDashboard.getNumber(fixedLeftLabel, kDefaultPower);
    rightPower = this.rightGain * SmartDashboard.getNumber(fixedRightLabel, kDefaultPower);
    mode = Mode.rampUp;
    ticks = 0;
  }

  @Override
  protected boolean isFinished() {
    return mode == Mode.done;
  }

  /**
   * Changes current mode of execute function when tick count is reached.
   * 
   * @param newMode   The mode to transition when the tick count is reached.
   * @param atTickCnt The tick count that triggers the mode change.
   */
  private void checkModeChange(Mode newMode, int atTickCnt) {
    if (ticks >= atTickCnt) {
      mode = newMode;
      ticks = 0;
    }
  }

  /**
   * Drive robot according to options specified during initialization.
   */
  @Override
  public void execute() {
    ticks++;

    switch (mode) {

    case rampUp: {
      // NOTE: To accelerate quickly we really want to allow the ramp power to go up to
      // a higher value than the cruise power and then change mode once we are near the
      // cruise velocity
      double rampPower = (double) ticks / (double) rampTicks;
      Robot.drive.setPower(leftPower * rampPower, rightPower * rampPower);
      checkModeChange(Mode.cruise, rampTicks);
      // Should see velocity ramp up to cruise velocity
      SmartDashboard.putNumber("Ramp Up Velocity (left)", Robot.drive.getLeft().getVelocity());
      SmartDashboard.putNumber("Ramp Up Velocity (right)", Robot.drive.getRight().getVelocity());
      break;
    }

    case cruise: {
      Robot.drive.setPower(leftPower, rightPower);
      checkModeChange(Mode.rampDown, cruiseTicks);
      // We hope these values remain close to the same
      SmartDashboard.putNumber("Cruise Velocity (left)", Robot.drive.getLeft().getVelocity());
      SmartDashboard.putNumber("Cruise Velocity (right)", Robot.drive.getRight().getVelocity());
      break;
    }

    case rampDown: {
      double rampPower = (double) (rampTicks - ticks) / (double) rampTicks;
      Robot.drive.setPower(leftPower * rampPower, rightPower * rampPower);
      checkModeChange(Mode.done, rampTicks);
      // We want this to be at zero
      SmartDashboard.putNumber("Ramp Down Velocity (left)", Robot.drive.getLeft().getVelocity());
      SmartDashboard.putNumber("Ramp Down Velocity (right)", Robot.drive.getRight().getVelocity());
      break;
    }

    case done:
    default: {
      Robot.drive.stop();
    }
    } // switch
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
