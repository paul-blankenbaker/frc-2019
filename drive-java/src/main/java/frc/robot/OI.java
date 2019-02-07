/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.drive.DriveMeasure;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  private final Joystick driver;

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


  /**
   * Install primary driver specific commands.
   */
  private void setupDriverCommands() {

  }

  private void setupDashboardCommands() {
    SmartDashboard.putData("Measure", new DriveMeasure());
  }
}
