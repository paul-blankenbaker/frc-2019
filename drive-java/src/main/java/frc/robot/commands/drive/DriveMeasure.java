package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.Tread;

/**
 * Utility command that measures drive distance and rotation (zeros each time it
 * is started).
 */
public class DriveMeasure extends Command {

  private DriveSubsystem drive;
  private Tread left;
  private Tread right;
  private double yawLast;
  private double leftDistLast;
  private int leftCntsLast;
  private double rightDistLast;
  private int rightCntsLast;

  public DriveMeasure() {
    super("Measure");
    // IMPORTANT. Since we are only reading data and not manipulting mechanisms, we
    // do not need
    // to require the drive subsystem
    drive = Robot.drive;
    left = drive.getLeft();
    right = drive.getRight();
    setRunWhenDisabled(true);
  }

  /**
   * Takes initial values of things measurements to "zero" each time the command
   * is started.
   * 
   */
  @Override
  protected void initialize() {
    yawLast = drive.getAngle();
    leftDistLast = left.getDistance();
    leftCntsLast = left.getCounts();
    rightDistLast = left.getDistance();
    rightCntsLast = right.getCounts();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double yawRaw = drive.getAngle();
    double yaw = yawRaw - yawLast;
    double leftDist = left.getDistance() - leftDistLast;
    int leftCnts = left.getCounts() - leftCntsLast;
    double leftVel = left.getVelocity();
    double rightDist = right.getDistance() - rightDistLast;
    int rightCnts = right.getCounts() - rightCntsLast;
    double rightVel = right.getVelocity();

    SmartDashboard.putNumber("Yaw NavX", yawRaw);
    SmartDashboard.putNumber("Yaw Measured", yaw);
    SmartDashboard.putNumber("Left Dist", leftDist);
    SmartDashboard.putNumber("Left Cnts", leftCnts);
    SmartDashboard.putNumber("Left Vel", leftVel);
    SmartDashboard.putNumber("Right Dist", rightDist);
    SmartDashboard.putNumber("Right Cnts", rightCnts);
    SmartDashboard.putNumber("Right Vel", rightVel);

    SmartDashboard.putNumber("Accel X", Robot.drive.getAccelX());
    SmartDashboard.putNumber("Accel Y", Robot.drive.getAccelY());
    SmartDashboard.putBoolean("Bump", Robot.drive.bumpCheck(0.4, 0.4));
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
