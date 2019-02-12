package frc.robot;

import edu.wpi.first.hal.sim.DriverStationSim;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Performance;
import frc.robot.subsystems.DriveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final boolean debug = true;

  // Subsystem singletons
  public static final DriveSubsystem drive = new DriveSubsystem();

  // Operator Interface singleton
  public static OI oi;
  private Command autonCommand;

  // Used to check interation times when debug is enabled
  private Performance performance;
  private DriverStationSim dsSim;
  private Timer dsTimer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // It is expected/required that ALL subsystems are constructed/initialized
    // first!
    oi = new OI();

    if (debug) {
      performance = new Performance();
      SmartDashboard.putData("Measure Performance", performance);
      if (isSimulation()) {
        dsSim = new DriverStationSim();
        dsSim.setAutonomous(true);
        dsTimer = new Timer();
        dsTimer.start();
      }
    }
  }

  /**
   * We will override the main loop function in debug mode so that we can do some
   * performance measurements.
   */
  @Override
  protected void loopFunc() {
    if (debug) {
      double startTime = Timer.getFPGATimestamp();
      super.loopFunc();
      double endTime = Timer.getFPGATimestamp();
      performance.addMeasurement(endTime - startTime);
    } else {
      super.loopFunc();
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    if (dsSim != null) {
      double dur = dsTimer.get();
      SmartDashboard.putNumber("Game Time", dur);
      if (dsSim.getEnabled()) {
        if (dsSim.getAutonomous()) {
          if (dur >= 15.0) {
            dsSim.setAutonomous(false);
            dsSim.setEnabled(false);
            dsTimer.reset();
            SmartDashboard.putString("Game Mode", "Disabled");
          }
        } else {
          if (dur >= 30.0) {
            dsSim.setAutonomous(true);
            dsSim.setEnabled(false);
            dsTimer.reset();
            SmartDashboard.putString("Game Mode", "Disabled");
          }
        }
      } else {
        if (dur >= 2.0) {
          dsSim.setEnabled(true);
          dsTimer.reset();
          SmartDashboard.putString("Game Mode", dsSim.getAutonomous() ? "Auton" : "Teleop");
        }
      }
      dsSim.notifyNewData();
    }
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    // Cancel current commands running on subsystems (will force them back to default)
    Subsystem[] list = { drive };
    for (Subsystem s : list) {
      Command active = s.getCurrentCommand();
      if (active != null) {
        active.cancel();
      }
    }
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    autonCommand = oi.getSelectedAuton();
    autonCommand.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonCommand != null) {
      autonCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  // @Override
  // public void testPeriodic() {
  // }
}
