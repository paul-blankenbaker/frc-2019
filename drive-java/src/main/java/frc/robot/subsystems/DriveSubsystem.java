package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.drive.DriveHuman;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;;

/**
 * Manages the components of the drive base.
 */
public final class DriveSubsystem extends Subsystem {
    private static final boolean debug = true;

    public static final int kTimeout = 0;

    // Used for applying real-world corrections to encoder.
    // Step 1: Set all 3 values the same (10.0) deploy code
    // Step 2: Measure out fixed distance (10.0 feet)
    // Step 3: Set two measured constants to values reported by encoders
    public static final double kExpectedDistance = 10.00;
    public static final double kMeasuredLeft = 10.00;
    public static final double kMeasuredRight = 10.00;

    // Used to convert encoder counts to distance (ft) for 6 inch wheels
    public static final double kWheelDiametetIn = 6.0;
    public static final int kEncCountPerRev = 256;
    public static final double kLeftConv = (kWheelDiametetIn / 12.0) * (Math.PI / kEncCountPerRev)
            * (kExpectedDistance / kMeasuredLeft);
    public static final double kRightConv = (kWheelDiametetIn / 12.0) * (Math.PI / kEncCountPerRev)
            * (kExpectedDistance / kMeasuredRight);

    /**
     * Each side of the drive base contains a collection of motors and encoder
     * managed by this object.
     */
    public static final class Tread {
        private final WPI_TalonSRX motor;
        private final Encoder encoder;
        private final String name;

        // Counts last read from encoder
        private int encCnts;
        // Distance last read from encoder
        private double encDist;
        // Velocity last returned from encoder
        private double encVel;
        private WPI_VictorSPX[] followers;

        public void setPower(double power) {
            motor.set(power);
        }

        public double getDistance() {
            return encDist;
        }

        public int getCounts() {
            return encCnts;
        }

        public double getVelocity() {
            return encVel;
        }

        /**
         * Helper method to create and initialize a TalonSRX speed controller.
         * 
         * @param canId  - ID on CAN bus of TalonSRX.
         * @param invert - Pass true if motor output needs to be inverted.
         * @return Initialized motor controller.
         */
        private static WPI_TalonSRX createTalonSRX(int canId, boolean invert) {
            WPI_TalonSRX s = new WPI_TalonSRX(canId);
            s.configFactoryDefault(kTimeout);
            s.clearStickyFaults(kTimeout);
            s.setSafetyEnabled(false);
            s.setInverted(invert);
            s.configContinuousCurrentLimit(15, kTimeout); // 15 Amps per motor
            s.configPeakCurrentLimit(20, kTimeout); // 20 Amps during Peak Duration
            s.configPeakCurrentDuration(100, kTimeout); // Peak Current for max 100 ms
            s.enableCurrentLimit(true);
            s.configOpenloopRamp(0.2, kTimeout); // number of seconds from 0 to 1
            return s;
        }

        /**
         * Helper method to create and initialize a VictorSPX speed controller.
         * 
         * @param leader - The TalonSRX that the VictorSPX should follow.
         * @param canId  - ID on CAN bus of VictorSPX.
         * @param invert - Pass true if motor output needs to be inverted.
         * @return Initialized motor controller.
         */
        private static WPI_VictorSPX createVictorSPX(WPI_TalonSRX leader, int canId, boolean invert) {
            WPI_VictorSPX s = new WPI_VictorSPX(canId);
            s.configFactoryDefault(kTimeout);
            s.clearStickyFaults(kTimeout);
            s.setSafetyEnabled(false);
            s.setInverted(invert);
            s.follow(leader);
            return s;
        }

        private Tread(String tname, int t0, int v1, int v2, boolean invert, int chA, int chB, double cntsToFt) {
            name = tname;
            motor = Tread.createTalonSRX(t0, invert);
            motor.setName("Drive", tname + "Motor0");
            WPI_VictorSPX[] flist = {};
            if (!RobotBase.isSimulation()) {
                WPI_VictorSPX s1 = Tread.createVictorSPX(motor, v1, invert);
                s1.setName("Drive", tname + "Motor1");
                WPI_VictorSPX s2 = Tread.createVictorSPX(motor, v2, invert);
                s2.setName("Drive", tname + "Motor2");

                flist = new WPI_VictorSPX[] { s1, s1 };
            }
            followers = flist;

            setVoltageCompensation(12); // Map [-1.0, +1.0] power values to [-12.0, +12.0] volts by default
            setBrakeMode(false);

            encoder = new Encoder(chA, chB);
            encoder.setName("Drive", tname + "Enc");
            encoder.setDistancePerPulse(cntsToFt);
            encoder.setSamplesToAverage(10);
            readSensors();
        }

        private void readSensors() {
            encCnts = encoder.get();
            encDist = encoder.getDistance();
            encVel = encoder.getRate();
        }

        private void dashboardPeriodic() {
            if (debug) {
                SmartDashboard.putNumber(name + " Enc Distance", encoder.getDistance());
                SmartDashboard.putNumber(name + " Enc Counts", encoder.get());
                SmartDashboard.putNumber(name + " Enc Velocity", encoder.getRate());
            }
        }

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
        private void setVoltageCompensation(double voltage) {
            motor.configVoltageCompSaturation(voltage, kTimeout);
            motor.enableVoltageCompensation(true);
            // NOT sure, but I don't think we need to apply this configuration to followers
        }

        /**
         * Controls whether or not the motor controllers will brake or coast when power
         * is set to 0.0.
         * 
         * @param enable Pass true to enable brake mode, false to enable coast mode.
         */
        private void setBrakeMode(boolean enable) {
            NeutralMode mode = enable ? NeutralMode.Brake : NeutralMode.Coast;
            motor.setNeutralMode(mode);
            for (WPI_VictorSPX f : followers) {
                // Apply configuration to followers as well (not sure if this is required)
                f.setNeutralMode(mode);
            }
        }

    }

    private final AHRS navx;

    private final BuiltInAccelerometer accel;

    private final Tread left;

    private final Tread right;

    private final DifferentialDrive drive;

    private float yaw;

    private double accelX;

    private double accelY;

    private double accelZ;

    public DriveSubsystem() {
        super("Drive");

        navx = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);
        navx.setName("Drive", "NavX");
        accel = new BuiltInAccelerometer();
        accel.setName("Drive", "Accel");
        left = new Tread("Left", RobotMap.kCanDriveLeft0, RobotMap.kCanDriveLeft1, RobotMap.kCanDriveLeft2, true,
                RobotMap.kDioDriveLeftEncA, RobotMap.kDioDriveLeftEncB, kLeftConv);
        right = new Tread("Right", RobotMap.kCanDriveRight0, RobotMap.kCanDriveRight1, RobotMap.kCanDriveRight2, false,
                RobotMap.kDioDriveRightEncA, RobotMap.kDioDriveRightEncB, kRightConv);

        drive = new DifferentialDrive(left.motor, right.motor);
        drive.setName("Drive", "Differential");
        drive.setSafetyEnabled(false);
        drive.setDeadband(0.025);
        drive.setRightSideInverted(false);
        periodic();
    }

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
    public void setVoltageCompensation(double maxVoltsLeft, double maxVoltsRight) {
        left.setVoltageCompensation(maxVoltsLeft);
        right.setVoltageCompensation(maxVoltsRight);
    }

    /**
     * Controls whether or not the motor controllers will brake or coast when power
     * is set to 0.0.
     * 
     * @param enable Pass true to enable brake mode, false to enable coast mode.
     */
    public void setBrakeMode(boolean enable) {
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
    public void setPower(double leftPower, double rightPower) {
        left.setPower(leftPower);
        right.setPower(rightPower);
    }

    public void arcadeDrive(double throttle, double rotation, boolean squareInputs) {
        drive.arcadeDrive(throttle, rotation, squareInputs);
    }

    public void curvatureDrive(double throttle, double rotation, boolean quickTurn) {
        drive.curvatureDrive(throttle, rotation, quickTurn);
    }

    public void tankDrive(double leftPower, double rightPower, boolean squareInputs) {
        drive.tankDrive(leftPower, rightPower, squareInputs);
    }

    @Override
    public void periodic() {
        readSensors();
        dashboardPeriodic();
    }

    private void readSensors() {
        left.readSensors();
        right.readSensors();
        yaw = navx.getYaw();
        accelX = accel.getX();
        accelY = accel.getY();
        accelZ = accel.getZ();
    }

    /**
     * Use this method if you need specific data or control of the left side of the
     * drive.
     */
    public Tread getLeft() {
        return left;
    }

    /**
     * Use this method if you need specific data or control of the right side of the
     * drive.
     * 
     * @return Right side of drive train.
     */
    public Tread getRight() {
        return right;
    }

    public double getAngle() {
        return yaw;
    }

    public double getAccelX() {
        return accelX;
    }

    public double getAccelY() {
        return accelY;
    }

    public double getAccelZ() {
        return accelZ;
    }

    public double getAvgDistance() {
        return (left.getDistance() + right.getDistance()) / 2;
    }

    public double getAvgVelocity() {
        return (left.getVelocity() + right.getVelocity()) / 2;
    }

    public double getAvgAbsVelocity() {
        return (Math.abs(left.getVelocity()) + Math.abs(right.getVelocity())) / 2;
    }

    public void stop() {
        setPower(0, 0);
    }

    private void dashboardPeriodic() {
        left.dashboardPeriodic();
        right.dashboardPeriodic();
    }

    /**
     * Returns true if magnitude of acceleration from built in accelerometer is
     * greater than bump limit specified.
     *
     * @param bumpX - Threshold for triggering bump condition in X axis.
     * @param bumpY - Threshold for triggering bump condition in Y axis.
     * @return true If either threshold was met.
     */
    public boolean bumpCheck(double bumpX, double bumpY) {
        boolean xBump = Math.abs(getAccelX()) > bumpX;
        boolean yBump = Math.abs(getAccelY()) > bumpY;
        return (xBump || yBump);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new DriveHuman());
    }
}
