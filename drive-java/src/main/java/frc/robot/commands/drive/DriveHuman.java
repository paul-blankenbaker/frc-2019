/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * An example command. You can replace me with your own command.
 */
public class DriveHuman extends Command {
  private static boolean isFlipped = false;
  private static boolean brakeMode = false;

  private static SendableChooser<Integer> modeChooser = null;

  // Drive mode choices
  private static final int kModeArcade = 0;
  private static final int kModeTank = 1;
  private static final int kModeCurvature = 2;
  private static final int kModeFixed = 3;

  private static final int kThrottleAxis = 1;
  private static final int kRotationAxis = 4;
  private static final int kLeftAxis = 1;
  private static final int kRightAxis = 3;

  private boolean squareInputs;

  private boolean quickTurn;

  private int mode;

  private double fixedLeft;

  private double fixedRight;

  private double rotationGain;

  private double slowGain;

  private double minDeflect;

  private JoystickButton flipButton;

  /**
   * Trivial command to allow user to control what end of the robot is the front.
   */
  private static final class FlipFront extends Command {
    FlipFront() {
      updateDashboard();
    }

    private void updateDashboard() {
      SmartDashboard.putBoolean("Flipped Front", isFlipped);
    }

    @Override
    protected void initialize() {
      // Change state and publish new state
      isFlipped = !isFlipped;
      updateDashboard();
    }

    @Override
    protected boolean isFinished() {
      return true;
    }
  }

  /**
   * Trivial helper command to allow user to control whether brake mode is enabled or not.
   */
  private static final class BrakeModeToggle extends Command {
    BrakeModeToggle() {
      updateDashboard();
    }

    private void updateDashboard() {
      SmartDashboard.putBoolean("Brake Mode", brakeMode);
    }

    @Override
    protected void initialize() {
      // Change state and publish new state
      brakeMode = !brakeMode;
      updateDashboard();
    }

    @Override
    protected boolean isFinished() {
      return true;
    }
  }


  /**
   * Initialize all of the control options and control option UI.
   */
  public DriveHuman() {
    super("HumanDrive", Robot.drive);
    squareInputs = true;
    quickTurn = false;
    mode = kModeArcade;
    fixedLeft = 0.4;
    fixedRight = 0.4;
    rotationGain = 0.5;
    slowGain = 0.5;
    minDeflect = 1 / 64.0;

    // Only need to set up dashbard drive controls once
    if (modeChooser == null) {
      SmartDashboard.putBoolean("Quick Turn", quickTurn);
      SmartDashboard.putBoolean("Square Inputs", squareInputs);
      SmartDashboard.putNumber("Fixed Left", fixedLeft);
      SmartDashboard.putNumber("Fixed Right", fixedRight);
      SmartDashboard.putNumber("Rotation Gain", rotationGain);
      SmartDashboard.putNumber("Slow Gain", slowGain);

      SendableChooser<Integer> mc = new SendableChooser<>();
      mc.setDefaultOption("Arcade", kModeArcade);
      mc.addOption("Tank", kModeTank);
      mc.addOption("Curvature", kModeCurvature);
      mc.addOption("Fixed", kModeFixed);
      SmartDashboard.putData("Drive Mode", mc);
      modeChooser = mc;

      SmartDashboard.putData("Brake Control", new BrakeModeToggle());
      SmartDashboard.putData("Flip Front", new FlipFront());
      flipButton = Robot.oi.createDriverButton(5);
      flipButton.whenPressed(new FlipFront());
    }
  }

  /**
   * Read in and apply current drive choices.
   */
  @Override
  public void initialize() {
    Robot.drive.setBrakeMode(brakeMode);
    mode = ((Number) modeChooser.getSelected()).intValue();
    quickTurn = SmartDashboard.getBoolean("Quick Turn", quickTurn);
    squareInputs = SmartDashboard.getBoolean("Square Inputs", squareInputs);
    fixedLeft = SmartDashboard.getNumber("Fixed Left", fixedLeft);
    fixedRight = SmartDashboard.getNumber("Fixed Right", fixedRight);
    rotationGain = SmartDashboard.getNumber("Rotation Gain", rotationGain);
    slowGain = SmartDashboard.getNumber("Slow Gain", slowGain);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  /**
   * Drive robot according to options specified during initialization.
   */
  @Override
  public void execute() {
    double gain = 1.0;
    double rotGain = rotationGain;

    // Slow-mo mode when user is holding down on button 6
    if (Robot.oi.readDriverButton(6)) {
      gain = slowGain;
      rotGain = slowGain;
    }

    switch (mode) {

    case kModeArcade: {
      double throttle = -Robot.oi.readDriverAxis(kThrottleAxis) * gain;
      double rotation = Robot.oi.readDriverAxis(kRotationAxis) * rotGain;
      if (isFlipped) {
        throttle = -throttle;
      }
      Robot.drive.arcadeDrive(throttle, rotation, squareInputs);
      break;
    }

    case kModeTank: {
      double left = -Robot.oi.readDriverAxis(kLeftAxis) * gain;
      double right = -Robot.oi.readDriverAxis(kRightAxis) * gain;
      if (isFlipped) {
        double tmpLeft = -left;
        left = -right;
        right = tmpLeft;
      }
      Robot.drive.tankDrive(left, right, squareInputs);
      break;
    }

    case kModeCurvature: {
      double throttle = -Robot.oi.readDriverAxis(kThrottleAxis) * gain;
      double rotation = Robot.oi.readDriverAxis(kRotationAxis) * rotGain;
      if (isFlipped) {
        throttle = -throttle;
      }
      Robot.drive.curvatureDrive(throttle, rotation, quickTurn);
      break;
    }

    case kModeFixed:
      driveFixed(gain, rotGain);
      break;

    default:
      // Unknown mode, stop driving
      Robot.drive.stop();
    }
  }

  /**
       * Drives left and right side motors at fixed power value specified on the dashboard.
       * @param gain - Gain multiplier for front/back.
       * @param rotGain - Gain multiplier for rotation.
       */
  private void driveFixed(double gain, double rotGain) {
    double leftPower = 0;
    double rightPower = 0;
    double throttle = -Robot.oi.readDriverAxis(kThrottleAxis);
    double rotation = Robot.oi.readDriverAxis(kRotationAxis);
      if (isFlipped) {
          throttle = -throttle;
      }

      if (throttle > minDeflect) {
          leftPower = fixedLeft;
          rightPower = fixedRight;
      } else if (throttle < -minDeflect) {
          leftPower = -fixedLeft;
          rightPower = -fixedRight;
      }  else {
          // No forward/reverse throttle, check for rotation
          gain = rotGain;
          if (rotation > minDeflect) {
              leftPower = fixedLeft;
              rightPower = -fixedRight;
          } else if (rotation < -minDeflect) {
              leftPower = -fixedLeft;
              rightPower = fixedRight;
          }
      }
      
      // Apply determined power value
      Robot.drive.setPower(leftPower * gain, rightPower * gain);
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
