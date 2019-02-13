#include "OI.h"
#include "Robot.h"
#include <frc/buttons/JoystickButton.h>
#include <frc/commands/WaitCommand.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "commands/LoadTest.h"
#include "commands/drive/DriveMeasure.h"
#include "commands/drive/DriveTickTimed.h"

using namespace frc;

OI::OI() : driver(0) {
}

bool OI::initializeBoolean(const char* dashboardLabel, bool defaultValue) {
  bool value = SmartDashboard::GetBoolean(dashboardLabel, defaultValue);
  SmartDashboard::PutBoolean(dashboardLabel, value);
  return value;
}

double OI::initializeNumber(const char* dashboardLabel, double defaultValue) {
  double value = SmartDashboard::GetNumber(dashboardLabel, defaultValue);
  SmartDashboard::PutNumber(dashboardLabel, value);
  return value;
}

void OI::installCommands() {
}

void OI::setupDriverCommands() {
  Joystick* stick = getDriverJoystick();
  JoystickButton flipButton(stick, 5);
  flipButton.WhenPressed(DriveHuman::createFlipFrontCommand());
}

void OI::setupDashboardCommands() {
  // Set up auton chooser
  autonChooser.SetDefaultOption("Do Nothing", new WaitCommand("Do Nothing", 3.0));
  autonChooser.AddOption("Drive Forward", new DriveTickTimed(1.0, 1.0));
  autonChooser.AddOption("Drive Backward", new DriveTickTimed(-1.0, -1.0));
  autonChooser.AddOption("Rotate Right", new DriveTickTimed(1.0, -1.0));
  autonChooser.AddOption("Rotate Left", new DriveTickTimed(-1.0, 1.0));
  SmartDashboard::PutData("Auto mode", &autonChooser);

  // Set up utility controls
  SmartDashboard::PutData("Measure", new DriveMeasure());

  // Drive controls
  DriveHuman::setupDashboardControls();
  SmartDashboard::PutData("Brake Control", DriveHuman::createBrakeModeToggleCommand());
  SmartDashboard::PutData("Flip Front", DriveHuman::createFlipFrontCommand());

  // Debug tools (if enabled)
  if (debug) {
    SmartDashboard::PutData("CPU Load Test", new LoadTest());
    SmartDashboard::PutData("Drive Subsystem", &Robot::drive);
    DifferentialDrive& dd = Robot::drive.getDifferentialDrive();
    if (&dd != 0) {
      SmartDashboard::PutData("DifferentialDrive", &dd);
    }
  }
}