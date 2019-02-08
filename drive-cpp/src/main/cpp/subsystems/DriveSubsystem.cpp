#include "subsystems/DriveSubsystem.h"
#include "commands/drive/DriveHuman.h"
#include <frc/RobotBase.h>
#include <frc/SPI.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

using namespace std;
using namespace frc;

//
// Implementation of Tread methods not found in header
//

WPI_TalonSRX* DriveSubsystem::Tread::createTalonSRX(int canId, bool invert) {
  WPI_TalonSRX* s = new WPI_TalonSRX(canId);
  s->ConfigFactoryDefault(kTimeout);
  s->ClearStickyFaults(kTimeout);
  s->SetSafetyEnabled(false);
  s->SetInverted(invert);
  s->ConfigContinuousCurrentLimit(15, kTimeout);  // 15 Amps per motor
  s->ConfigPeakCurrentLimit(20, kTimeout);        // 20 Amps during Peak Duration
  s->ConfigPeakCurrentDuration(100, kTimeout);    // Peak Current for max 100 ms
  s->EnableCurrentLimit(true);
  s->ConfigOpenloopRamp(0.2, kTimeout);  // number of seconds from 0 to 1
  return s;
}

WPI_VictorSPX* DriveSubsystem::Tread::createVictorSPX(WPI_TalonSRX* leader, int canId, bool invert) {
  WPI_VictorSPX* s = new WPI_VictorSPX(canId);
  s->ConfigFactoryDefault(kTimeout);
  s->ClearStickyFaults(kTimeout);
  s->SetSafetyEnabled(false);
  s->SetInverted(invert);
  s->Follow(*leader);
  return s;
}

DriveSubsystem::Tread::Tread(string tname, int t0, int v1, int v2, bool invert, int chA, int chB, double cntsToFt)
    : encoder(chA, chB) {
  name = tname;
  motor = createTalonSRX(t0, invert);
  motor->SetName("Drive", tname + "Motor0");
  if (RobotBase::IsSimulation()) {
    for (int i = 0; i < kNumFollowers; i++) {
      followers[i] = 0;
    }
  } else {
    WPI_VictorSPX* s1 = createVictorSPX(motor, v1, invert);
    s1->SetName("Drive", tname + "Motor1");
    followers[0] = s1;
    WPI_VictorSPX* s2 = createVictorSPX(motor, v2, invert);
    s2->SetName("Drive", tname + "Motor2");
    followers[1] = s2;
  }

  setVoltageCompensation(12);  // Map [-1.0, +1.0] power values to [-12.0, +12.0] volts by default
  setBrakeMode(false);

  encoder.SetName("Drive", tname + "Enc");
  encoder.SetDistancePerPulse(cntsToFt);
  encoder.SetSamplesToAverage(10);
  readSensors();
}

DriveSubsystem::Tread::~Tread() {
  // Clean up objects that were allocated with new
  delete motor;
  for (int i = 0; i < kNumFollowers; i++) {
    delete followers[i];
  }
}

void DriveSubsystem::Tread::dashboardPeriodic() {
  if (kDebug) {
    SmartDashboard::PutNumber(name + " Enc Distance", encoder.GetDistance());
    SmartDashboard::PutNumber(name + " Enc Counts", encoder.Get());
    SmartDashboard::PutNumber(name + " Enc Velocity", encoder.GetRate());
  }
}

void DriveSubsystem::Tread::setVoltageCompensation(double voltage) {
  motor->ConfigVoltageCompSaturation(voltage, kTimeout);
  motor->EnableVoltageCompensation(true);
  // NOT sure, but I don't think we need to apply this configuration to followers
}

void DriveSubsystem::Tread::setBrakeMode(bool enable) {
  NeutralMode mode = enable ? NeutralMode::Brake : NeutralMode::Coast;
  motor->SetNeutralMode(mode);
  for (int i = 0; i < kNumFollowers; i++) {
    // Apply configuration to followers as well (not sure if this is required)
    WPI_VictorSPX* f = followers[i];
    f->SetNeutralMode(mode);
  }
}

//
// Implementation of DriveSubsystem methods not found in header
//

DriveSubsystem::DriveSubsystem() : Subsystem("Drive"),
                                   navx(SPI::Port::kMXP),
                                   accel(),
                                   left("Left", kCanDriveLeft0, kCanDriveLeft1, kCanDriveLeft2, true,
                                        kDioDriveLeftEncA, kDioDriveLeftEncB, kLeftConv),
                                   right("Right", kCanDriveRight0, kCanDriveRight1, kCanDriveRight2, false,
                                         kDioDriveRightEncA, kDioDriveRightEncB, kRightConv) {
  navx.SetName("Drive", "NavX");

  accel.SetName("Drive", "Accel");

  drive = new DifferentialDrive(*left.motor, *right.motor);
  drive->SetName("Drive", "Differential");
  drive->SetSafetyEnabled(false);
  drive->SetDeadband(0.025);
  drive->SetRightSideInverted(false);
  periodic();
}

DriveSubsystem::~DriveSubsystem() {
  // Clean up objects that were memory allocated
  delete drive;
}

void DriveSubsystem::InitDefaultCommand() {
  SetDefaultCommand(new DriveHuman());
}

void DriveSubsystem::periodic() {
  readSensors();
  dashboardPeriodic();
}
void DriveSubsystem::dashboardPeriodic() {
  left.dashboardPeriodic();
  right.dashboardPeriodic();
}

void DriveSubsystem::readSensors() {
  left.readSensors();
  right.readSensors();
  yaw = navx.GetYaw();
  accelX = accel.GetX();
  accelY = accel.GetY();
  accelZ = accel.GetZ();
}

double DriveSubsystem::getAvgAbsVelocity() const {
  return (abs(left.getVelocity()) + abs(right.getVelocity())) / 2;
}

bool DriveSubsystem::bumpCheck(double bumpX, double bumpY) const {
  bool xBump = abs(getAccelX()) > bumpX;
  bool yBump = abs(getAccelY()) > bumpY;
  return (xBump || yBump);
}
