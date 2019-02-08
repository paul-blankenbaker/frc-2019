#pragma once

#include <AHRS.h>
#include <ctre/Phoenix.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/Encoder.h>
#include <frc/commands/Subsystem.h>
#include <frc/drive/DifferentialDrive.h>

#include "RobotMap.h"

class DriveSubsystem : public frc::Subsystem {
 public:
  /**
   * Helper class to manage one set of motors and encoder (drive base has two of these).
   */
  class Tread {
   private:
    static constexpr int kNumFollowers = 2;

    WPI_TalonSRX* motor;
    frc::Encoder encoder;
    std::string name;
    WPI_VictorSPX* followers[kNumFollowers];

    // Counts last read from encoder
    int encCnts;
    // Distance last read from encoder
    double encDist;
    // Velocity last returned from encoder
    double encVel;

   public:
    void setPower(double power) {
      motor->Set(power);
    }

    double getDistance() const {
      return encDist;
    }

    int getCounts() const {
      return encCnts;
    }

    double getVelocity() const {
      return encVel;
    }

   private:
    /**
     * Helper method to create and initialize a TalonSRX speed controller.
     * 
     * @param canId  - ID on CAN bus of TalonSRX.
     * @param invert - Pass true if motor output needs to be inverted.
     * @return Initialized motor controller.
     */
    static WPI_TalonSRX* createTalonSRX(int canId, bool invert);

    /**
     * Helper method to create and initialize a VictorSPX speed controller.
     * 
     * @param leader - The TalonSRX that the VictorSPX should follow.
     * @param canId  - ID on CAN bus of VictorSPX.
     * @param invert - Pass true if motor output needs to be inverted.
     * @return Initialized motor controller.
     */
    static WPI_VictorSPX* createVictorSPX(WPI_TalonSRX* leader, int canId, bool invert);

    Tread(std::string tname, int t0, int v1, int v2, bool invert, int chA, int chB, double cntsToFt);
    ~Tread();

    void readSensors() {
      encCnts = encoder.Get();
      encDist = encoder.GetDistance();
      encVel = encoder.GetRate();
    }

    void dashboardPeriodic();

    /**
     * Sets the voltage range that [-1.0, +1.0] maps to.
     * 
     * <p>
     * For consistent auton, you probably want to set this value to just under the
     * lowest value you see your battery voltage drop to when the robot is running.
     * </p>
     * 
     * @param voltage Maximum voltage to use when -1 or +1 is specified as power
     *                level (typically 12.0 or less).
     */
    void setVoltageCompensation(double voltage);

    /**
     * Controls whether or not the motor controllers will brake or coast when power
     * is set to 0.0.
     * 
     * @param enable Pass true to enable brake mode, false to enable coast mode.
     */
    void setBrakeMode(bool enable);

    // Allow DriveSubsystem access to Tread's private methods
    friend class DriveSubsystem;
  };

  //
  // Private data and methods in the DriveSubsystem class
  //
 private:
  static constexpr bool kDebug = true;

  static constexpr int kTimeout = 0;

  // Used for applying real-world corrections to encoder.
  // Step 1: Set all 3 values the same (10.0) deploy code
  // Step 2: Measure out fixed distance (10.0 feet)
  // Step 3: Set two measured constants to values reported by encoders
  static constexpr double kExpectedDistance = 10.00;
  static constexpr double kMeasuredLeft = 10.00;
  static constexpr double kMeasuredRight = 10.00;

  // Used to convert encoder counts to distance (ft) for 6 inch wheels
  static constexpr double kWheelDiametetIn = 6.0;
  static constexpr int kEncCountPerRev = 256;
  static constexpr double kLeftConv = (kWheelDiametetIn / 12.0) * (Math_PI / kEncCountPerRev) * (kExpectedDistance / kMeasuredLeft);
  static constexpr double kRightConv = (kWheelDiametetIn / 12.0) * (Math_PI / kEncCountPerRev) * (kExpectedDistance / kMeasuredRight);

  AHRS navx;

  frc::BuiltInAccelerometer accel;

  Tread left;

  Tread right;

  frc::DifferentialDrive* drive;

  double yaw;

  double accelX;

  double accelY;

  double accelZ;

  void readSensors();

  void dashboardPeriodic();

  //
  // Public methods in the DriveSubsystem class
  //
 public:
  DriveSubsystem();
  ~DriveSubsystem();

  void InitDefaultCommand() override;

  /**
     * Sets the voltage range that [-1.0, +1.0] maps to.
     * 
     * <p>
     * For consistent auton, you probably want to set this value to just under the
     * lowest value you see your battery voltage drop to when the robot is running.
     * </p>
     * 
     * <p>
     * You will normally specify the same value for both sides, however if your
     * robot pulls hard to one side or the other you can drop the limit to the more
     * powerful side to apply a linear correction gain to get it closer to straight.
     * </p>
     * 
     * @param maxVoltsLeft  Maximum voltage to use for the left side of the drive
     *                      train when -1 or +1 is specified as power level
     *                      (typically 12.0 or less).
     * @param maxVoltsRight Maximum voltage to use for the right side of the drive
     *                      train when -1 or +1 is specified as power level
     *                      (typically 12.0 or less).
     */
  void setVoltageCompensation(double maxVoltsLeft, double maxVoltsRight) {
    left.setVoltageCompensation(maxVoltsLeft);
    right.setVoltageCompensation(maxVoltsRight);
  }

  /**
     * Controls whether or not the motor controllers will brake or coast when power
     * is set to 0.0.
     * 
     * @param enable Pass true to enable brake mode, false to enable coast mode.
     */
  void setBrakeMode(bool enable) {
    left.setBrakeMode(enable);
    right.setBrakeMode(enable);
  }

  /**
     * Use in autonomous commands to directly apply power to the left an right
     * motors.
     * 
     * @param leftPower  - Power for left side in range [-1.0, +1.0]. Positive moves
     *                   forward.
     * @param rightPower - Power for right side in range [-1.0, +1.0]. Positive
     *                   moves forward.
     */
  void setPower(double leftPower, double rightPower) {
    left.setPower(leftPower);
    right.setPower(rightPower);
  }

  void arcadeDrive(double throttle, double rotation, bool squareInputs) {
    drive->ArcadeDrive(throttle, rotation, squareInputs);
  }

  void curvatureDrive(double throttle, double rotation, bool quickTurn) {
    drive->CurvatureDrive(throttle, rotation, quickTurn);
  }

  void tankDrive(double leftPower, double rightPower, bool squareInputs) {
    drive->TankDrive(leftPower, rightPower, squareInputs);
  }

  void periodic();

  /**
     * Use this method if you need specific data or control of the left side of the
     * drive.
     */
  Tread& getLeft() {
    return left;
  }

  /**
     * Use this method if you need specific data or control of the right side of the
     * drive.
     * 
     * @return Right side of drive train.
     */
  Tread& getRight() {
    return right;
  }

  double getAngle() const {
    return yaw;
  }

  double getAccelX() const {
    return accelX;
  }

  double getAccelY() const {
    return accelY;
  }

  double getAccelZ() const {
    return accelZ;
  }

  double getAvgDistance() const {
    return (left.getDistance() + right.getDistance()) / 2;
  }

  double getAvgVelocity() const {
    return (left.getVelocity() + right.getVelocity()) / 2;
  }

  double getAvgAbsVelocity() const;

  void stop() {
    setPower(0, 0);
  }

  /**
     * Returns true if magnitude of acceleration from built in accelerometer is
     * greater than bump limit specified.
     *
     * @param bumpX - Threshold for triggering bump condition in X axis.
     * @param bumpY - Threshold for triggering bump condition in Y axis.
     * @return true If either threshold was met.
     */
  bool bumpCheck(double bumpX, double bumpY) const;
};
