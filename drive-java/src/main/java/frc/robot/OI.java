package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LoadTest;
import frc.robot.commands.drive.DriveHuman;
import frc.robot.commands.drive.DriveMeasure;
import frc.robot.commands.drive.DriveTickTimed;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  private static final boolean debug = true;

  private final Joystick driver;

  public static boolean initializeBoolean(String dashboardLabel, boolean defaultValue) {
    boolean value = SmartDashboard.getBoolean(dashboardLabel, defaultValue);
    SmartDashboard.putBoolean(dashboardLabel, value);
    return value;
  }

  public static double initializeNumber(String dashboardLabel, double defaultValue) {
    double value = SmartDashboard.getNumber(dashboardLabel, defaultValue);
    SmartDashboard.putNumber(dashboardLabel, value);
    return value;
  }

  public JoystickButton createDriverButton(int buttonId) {
    return new JoystickButton(driver, buttonId);
  }

  public boolean readDriverButton(int buttonId) {
    return driver.getRawButton(buttonId);
  }

  public double readDriverAxis(int axisId) {
    return driver.getRawAxis(axisId);
  }

  OI() {
    driver = new Joystick(0);

    setupDriverCommands();
    setupDashboardCommands();
  }

  private SendableChooser<Command> autonChooser;

  Command getSelectedAuton() {
    return autonChooser.getSelected();
  }

  /**
   * Install primary driver specific commands.
   */
  private void setupDriverCommands() {
    JoystickButton flipButton = createDriverButton(5);
    flipButton.whenPressed(DriveHuman.createFlipFrontCommand());
  }

  private void setupDashboardCommands() {
    // Set up auton chooser
    autonChooser = new SendableChooser<>();
    autonChooser.setDefaultOption("Do Nothing", new WaitCommand("Do Nothing", 3.0));
    autonChooser.addOption("Drive Forward", new DriveTickTimed(1.0, 1.0));
    autonChooser.addOption("Drive Backward", new DriveTickTimed(-1.0, -1.0));
    autonChooser.addOption("Rotate Right", new DriveTickTimed(1.0, -1.0));
    autonChooser.addOption("Rotate Left", new DriveTickTimed(-1.0, 1.0));
    SmartDashboard.putData("Auto mode", autonChooser);

    // Set up utility controls
    SmartDashboard.putData("Measure", new DriveMeasure());

    // Drive controls
    DriveHuman.setupDashboardControls();
    SmartDashboard.putData("Brake Control", DriveHuman.createBrakeModeToggleCommand());
    SmartDashboard.putData("Flip Front", DriveHuman.createFlipFrontCommand());

    // Debug tools (if enabled)
    if (debug) {
      SmartDashboard.putData("CPU Load Test", new LoadTest());
      SmartDashboard.putData("Drive Subsystem", Robot.drive);
      DifferentialDrive dd = Robot.drive.getDifferentialDrive();
      if (dd != null) {
        SmartDashboard.putData("DifferentialDrive", dd);
      }
    }
  }
}
