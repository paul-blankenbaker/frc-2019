package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.math.Stats;

/**
* Used to measure the performance time it takes for the main loop to run.
*/
public class Performance extends Command {
  private Stats stats;
  private double lastTime;
  
  public Performance() {
    stats = new Stats();
    this.setRunWhenDisabled(true);
  }
  
  public void addMeasurement(double timeSecs) {
    lastTime = timeSecs;
    stats.add(timeSecs);
  }
  
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    lastTime = 0.0;
    stats.zero();
  }
  
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("Run Last", lastTime);
    SmartDashboard.putNumber("Run Avg", stats.getAvg());
    SmartDashboard.putNumber("Run Max", stats.getMax());
  }
  
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }
  
}
