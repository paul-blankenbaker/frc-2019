#include "commands/drive/DriveMeasure.h"
#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

DriveMeasure::DriveMeasure() : Command("Measure") {
  // IMPORTANT. Since we are only reading data and not manipulting mechanisms, we
  // do not need
  // to require the drive subsystem
  SetRunWhenDisabled(true);
}

void DriveMeasure::Initialize() {
  DriveSubsystem& drive = Robot::drive;
  DriveSubsystem::Tread& left = drive.getLeft();
  DriveSubsystem::Tread& right = drive.getRight();

  yawLast = drive.getAngle();
  leftDistLast = left.getDistance();
  leftCntsLast = left.getCounts();
  rightDistLast = left.getDistance();
  rightCntsLast = right.getCounts();
}

void DriveMeasure::Execute() {
  DriveSubsystem& drive = Robot::drive;
  DriveSubsystem::Tread& left = drive.getLeft();
  DriveSubsystem::Tread& right = drive.getRight();

  double yawRaw = drive.getAngle();
  double yaw = yawRaw - yawLast;
  double leftDist = left.getDistance() - leftDistLast;
  int leftCnts = left.getCounts() - leftCntsLast;
  double leftVel = left.getVelocity();
  double rightDist = right.getDistance() - rightDistLast;
  int rightCnts = right.getCounts() - rightCntsLast;
  double rightVel = right.getVelocity();

  SmartDashboard::PutNumber("Yaw NavX", yawRaw);
  SmartDashboard::PutNumber("Yaw Measured", yaw);
  SmartDashboard::PutNumber("Left Dist", leftDist);
  SmartDashboard::PutNumber("Left Cnts", leftCnts);
  SmartDashboard::PutNumber("Left Vel", leftVel);
  SmartDashboard::PutNumber("Right Dist", rightDist);
  SmartDashboard::PutNumber("Right Cnts", rightCnts);
  SmartDashboard::PutNumber("Right Vel", rightVel);

  SmartDashboard::PutNumber("Accel X", drive.getAccelX());
  SmartDashboard::PutNumber("Accel Y", drive.getAccelY());
  SmartDashboard::PutBoolean("Bump", drive.bumpCheck(0.4, 0.4));
}

bool DriveMeasure::IsFinished() {
  return false;
}
