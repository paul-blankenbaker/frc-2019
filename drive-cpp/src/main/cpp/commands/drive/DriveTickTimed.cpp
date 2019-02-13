#include <Robot.h>
#include <commands/drive/DriveTickTimed.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

DriveTickTimed::DriveTickTimed(double leftGain, double rightGain) : Command("DriveTickTimed", Robot::drive) {
  this->leftGain = leftGain;
  this->rightGain = rightGain;
  leftPower = rightPower = kDefaultPower;
  mode = Mode::done;
  rampTicks = 20;
  cruiseTicks = 40;

  leftPower = OI::initializeNumber(fixedLeftLabel, leftPower);
  rightPower = OI::initializeNumber(fixedRightLabel, rightPower);
  rampTicks = (int)OI::initializeNumber(rampTicksLabel, rampTicks);
  cruiseTicks = (int)OI::initializeNumber(cruiseTicksLabel, cruiseTicks);
}

void DriveTickTimed::Initialize() {
  rampTicks = (int)SmartDashboard::GetNumber(rampTicksLabel, rampTicks);
  cruiseTicks = (int)SmartDashboard::GetNumber(cruiseTicksLabel, cruiseTicks);
  leftPower = leftGain * SmartDashboard::GetNumber(fixedLeftLabel, kDefaultPower);
  rightPower = rightGain * SmartDashboard::GetNumber(fixedRightLabel, kDefaultPower);
  mode = Mode::rampUp;
  ticks = 0;
}

bool DriveTickTimed::IsFinished() {
  return mode == Mode::done;
}

void DriveTickTimed::checkModeChange(Mode newMode, int atTickCnt) {
  if (ticks >= atTickCnt) {
    mode = newMode;
    ticks = 0;
  }
}

void DriveTickTimed::Execute() {
  ticks++;

  switch (mode) {
    case rampUp: {
      // NOTE: To accelerate quickly we really want to allow the ramp power to go up to
      // a higher value than the cruise power and then change mode once we are near the
      // cruise velocity
      double rampPower = (double)ticks / (double)rampTicks;
      Robot::drive.setPower(leftPower * rampPower, rightPower * rampPower);
      checkModeChange(Mode::cruise, rampTicks);
      // Should see velocity ramp up to cruise velocity
      SmartDashboard::PutNumber("Ramp Up Velocity (left)", Robot::drive.getLeft().getVelocity());
      SmartDashboard::PutNumber("Ramp Up Velocity (right)", Robot::drive.getRight().getVelocity());
      break;
    }

    case cruise: {
      Robot::drive.setPower(leftPower, rightPower);
      checkModeChange(Mode::rampDown, cruiseTicks);
      // We hope these values remain close to the same
      SmartDashboard::PutNumber("Cruise Velocity (left)", Robot::drive.getLeft().getVelocity());
      SmartDashboard::PutNumber("Cruise Velocity (right)", Robot::drive.getRight().getVelocity());
      break;
    }

    case rampDown: {
      double rampPower = (double)(rampTicks - ticks) / (double)rampTicks;
      Robot::drive.setPower(leftPower * rampPower, rightPower * rampPower);
      checkModeChange(Mode::done, rampTicks);
      // We want this to be at zero
      SmartDashboard::PutNumber("Ramp Down Velocity (left)", Robot::drive.getLeft().getVelocity());
      SmartDashboard::PutNumber("Ramp Down Velocity (right)", Robot::drive.getRight().getVelocity());
      break;
    }

    case done:
    default: {
      Robot::drive.stop();
    }
  }  // switch
}

void DriveTickTimed::End() {
  Robot::drive.stop();
}

void DriveTickTimed::Interrupted() {
  End();
}
