package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.math.Stats;

/**
 * Runs a set of operations to test performance.
 */
public final class LoadTest extends Command {

  private boolean previousBelow;
  private int loopCnts;
  private double runSecs;
  private Timer timer;
  private Stats stats;

  public LoadTest() {
    super("LoadTest");
    previousBelow = false;
    loopCnts = 8096;
    runSecs = 0.0;
    timer = new Timer();
    stats = new Stats();
    setRunWhenDisabled(true);
  }

  private double runTest(int cnt) {
    double accum = 0.0;
    for (int i = 0; i < cnt; i++) {
      int n = (i / 4);
      for (int j = 0; j < n; j++) {
        int x = ((j >> 3) + 1) * 7 / 8;
        if ((j % 4) == 0) {
          x = -x;
        }
        stats.add(x);
      }
      double ratio = (double) i / (double) cnt;
      accum += Math.sin(ratio);
      accum += Math.pow(Math.PI, ratio);
      accum += Math.cos(ratio);
      stats.add(accum);
    }

    return accum;
  }

  /**
   * Reset for next run.
   */
  @Override
  public void initialize() {
    previousBelow = false;
    runSecs = 0;
    loopCnts = 1;
    stats.zero();
  }

  /**
   * Measure how long it takes to run a bunch of iterations.
   */
  @Override
  public void execute() {
    SmartDashboard.putNumber("CPU Test Loops", loopCnts);
    timer.reset();
    timer.start();
    runTest(loopCnts);
    runSecs = timer.get();
  }

  /**
   * Keep running until we cross over from too short of time to too long of time.
   */
  @Override
  public boolean isFinished() {
    if (runSecs > 0.1) {
      if (previousBelow) {
        return true;
      }
      previousBelow = false;
      loopCnts = loopCnts / 2;
    } else {
      previousBelow = true;
      loopCnts = loopCnts * 2;
    }
    return false;
  }

  @Override
  public void end() {
    double hz = 0;
    if (runSecs > 0) {
      hz = loopCnts / runSecs;
    }
    SmartDashboard.putNumber("CPU Test Secs", runSecs);
    SmartDashboard.putNumber("CPU Test Hz", hz);
  }
}
