#include <Robot.h>
#include <commands/drive/DriveHuman.h>
#include <frc/buttons/JoystickButton.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

namespace {
// Indicates if driver wants front/back flipped while driving
bool isFlipped = false;
// Indicates if driver wants brake mode enabled
bool brakeMode = false;

// Only need to set up singleton one time
SendableChooser<int>* modeChooser = 0;

/**
 * Trivial command to allow user to control what end of the robot is the front.
 */
class FlipFront : public Command {
 private:
  void updateDashboard() {
    SmartDashboard::PutBoolean("Flipped Front", isFlipped);
  }

 public:
  FlipFront() : Command("FlipFront") {
    updateDashboard();
  }

  void Initialize() override {
    // Change state and publish new state
    isFlipped = !isFlipped;
    updateDashboard();
  }

  bool IsFinished() override {
    return true;
  }
};

/**
 * Trivial helper command to allow user to control whether brake mode is enabled or not.
 */
class BrakeModeToggle : public Command {
 private:
  void updateDashboard() {
    SmartDashboard::PutBoolean("Brake Mode", brakeMode);
  }

 public:
  BrakeModeToggle() : Command("BrakeModeToggle") {
    updateDashboard();
  }

 protected:
  void Initialize() override {
    // Change state and publish new state
    brakeMode = !brakeMode;
    updateDashboard();
  }

  bool IsFinished() override {
    return true;
  }
};
}  // namespace

//
// DriveHuman method implementation
//

DriveHuman::DriveHuman() : Command("HumanDrive", Robot::drive),
                           squareInputs(true),
                           quickTurn(false),
                           mode(kModeArcade),
                           fixedLeft(0.4),
                           fixedRight(0.4),
                           rotationGain(0.5),
                           slowGain(0.5),
                           minDeflect(1 / 64.0) {
}

Command* DriveHuman::createFlipFrontCommand() {
  return new FlipFront();
}

Command* DriveHuman::createBrakeModeToggleCommand() {
  return new BrakeModeToggle();
}

/**
   * Helper method that OI can use during setup to enable dashboard driver control settings.
   */
void DriveHuman::setupDashboardControls() {
  // Only need to set up dashbard drive controls once
  if (modeChooser == 0) {
    OI::initializeBoolean(quickTurnLabel, true);
    OI::initializeBoolean(squaredInputsLabel, true);
    OI::initializeNumber(fixedLeftLabel, 0.4);
    OI::initializeNumber(fixedRightLabel, 0.4);
    OI::initializeNumber(rotationGainLabel, 0.5);
    OI::initializeNumber(slowGainLabel, 0.5);

    SendableChooser<int>* mc = new SendableChooser<int>();
    mc->SetDefaultOption("Arcade", kModeArcade);
    mc->AddOption("Tank", kModeTank);
    mc->AddOption("Curvature", kModeCurvature);
    mc->AddOption("Fixed", kModeFixed);
    SmartDashboard::PutData(driveModeLabel, mc);
    modeChooser = mc;
  }
}

/**
 * Read in and apply current drive choices.
 */
void DriveHuman::Initialize() {
  Robot::drive.setBrakeMode(brakeMode);
  if (modeChooser != 0) {
    mode = modeChooser->GetSelected();
  }
  quickTurn = SmartDashboard::GetBoolean("Quick Turn", quickTurn);
  squareInputs = SmartDashboard::GetBoolean("Square Inputs", squareInputs);
  fixedLeft = SmartDashboard::GetNumber("Fixed Left", fixedLeft);
  fixedRight = SmartDashboard::GetNumber("Fixed Right", fixedRight);
  rotationGain = SmartDashboard::GetNumber("Rotation Gain", rotationGain);
  slowGain = SmartDashboard::GetNumber("Slow Gain", slowGain);
}

bool DriveHuman::IsFinished() {
  return false;
}

/**
 * Drive robot according to options specified during initialization.
 */
void DriveHuman::Execute() {
  double gain = 1.0;
  double rotGain = rotationGain;

  // Slow-mo mode when user is holding down on button 6
  if (Robot::oi.readDriverButton(6)) {
    gain = slowGain;
    rotGain = slowGain;
  }

  switch (mode) {
    case kModeArcade: {
      double throttle = -Robot::oi.readDriverAxis(kThrottleAxis) * gain;
      double rotation = Robot::oi.readDriverAxis(kRotationAxis) * rotGain;
      if (isFlipped) {
        throttle = -throttle;
      }
      DifferentialDrive& drive = Robot::drive.getDifferentialDrive();
      drive.ArcadeDrive(throttle, rotation, squareInputs);
      break;
    }

    case kModeTank: {
      double left = -Robot::oi.readDriverAxis(kLeftAxis) * gain;
      double right = -Robot::oi.readDriverAxis(kRightAxis) * gain;
      if (isFlipped) {
        double tmpLeft = -left;
        left = -right;
        right = tmpLeft;
      }
      DifferentialDrive& drive = Robot::drive.getDifferentialDrive();
      drive.TankDrive(left, right, squareInputs);
      break;
    }

    case kModeCurvature: {
      double throttle = -Robot::oi.readDriverAxis(kThrottleAxis) * gain;
      double rotation = Robot::oi.readDriverAxis(kRotationAxis) * rotGain;
      if (isFlipped) {
        throttle = -throttle;
      }
      DifferentialDrive& drive = Robot::drive.getDifferentialDrive();
      drive.CurvatureDrive(throttle, rotation, quickTurn);
      break;
    }

    case kModeFixed:
      driveFixed(gain, rotGain);
      break;

    default:
      // Unknown mode, stop driving
      Robot::drive.stop();
  }
}

/**
 * Drives left and right side motors at fixed power value specified on the dashboard.
 * 
 * @param gain - Gain multiplier for front/back.
 * @param rotGain - Gain multiplier for rotation.
 */
void DriveHuman::driveFixed(double gain, double rotGain) {
  double leftPower = 0;
  double rightPower = 0;
  double throttle = -Robot::oi.readDriverAxis(kThrottleAxis);
  double rotation = Robot::oi.readDriverAxis(kRotationAxis);
  if (isFlipped) {
    throttle = -throttle;
  }

  if (throttle > minDeflect) {
    leftPower = fixedLeft;
    rightPower = fixedRight;
  } else if (throttle < -minDeflect) {
    leftPower = -fixedLeft;
    rightPower = -fixedRight;
  } else {
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
  Robot::drive.setPower(leftPower * gain, rightPower * gain);
}

// Called once after isFinished returns true
void DriveHuman::End() {
  Robot::drive.stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveHuman::Interrupted() {
  End();
}
