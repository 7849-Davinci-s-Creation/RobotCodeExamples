package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import lib.Encoderutil;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class DriveTrain extends SubsystemBase {
    private final CANSparkMax leftLeader = new CANSparkMax(Constants.FRONTLEFTMOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(Constants.BACKLEFTMOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(Constants.FRONTRIGHTMOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(Constants.BACKRIGHTMOTOR_PORT, MotorType.kBrushless);

    private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
    private final RelativeEncoder rightEncoder = leftFollower.getEncoder();

    private final AHRS navX = new AHRS(SPI.Port.kMXP);
    private final DifferentialDriveOdometry odometry;

    private final MutableMeasure<Voltage> appliedVoltage = MutableMeasure.mutable(Volts.of(0));
    private final MutableMeasure<Distance> distance = MutableMeasure.mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> velocity = MutableMeasure.mutable(MetersPerSecond.of(0));

    SysIdRoutine.Config config = new SysIdRoutine.Config(Volts.of(3).per(Seconds.of(1)),
            Volts.of(3),
            Seconds.of(3),
            null);

    SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
            this::voltageDrive,
            this::log,
            this);

    SysIdRoutine driveTrainRoutine = new SysIdRoutine(config, mechanism);

    public DriveTrain() {
        navX.reset();
        resetEncoders();

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);
        leftLeader.setInverted(true);

        leftLeader.setIdleMode(IdleMode.kBrake);
        rightLeader.setIdleMode(IdleMode.kBrake);
        leftFollower.setIdleMode(IdleMode.kBrake);
        rightFollower.setIdleMode(IdleMode.kBrake);

        odometry = new DifferentialDriveOdometry(navX.getRotation2d(), leftEncoder.getPosition(),
                rightEncoder.getPosition());

        leftEncoder.setPositionConversionFactor(Encoderutil.neoEncoderLinearDistanceConversionFactorMeters(5.95, 3));
        rightEncoder.setPositionConversionFactor(Encoderutil.neoEncoderLinearDistanceConversionFactorMeters(5.95, 3));

        leftEncoder
                .setVelocityConversionFactor(Encoderutil.neoEncoderLinearDistanceConversionFactorMeters(5.95, 3) / 60);
        rightEncoder
                .setVelocityConversionFactor(Encoderutil.neoEncoderLinearDistanceConversionFactorMeters(5.95, 3) / 60);
    }

    public void voltageDrive(Measure<Voltage> volts) {
        leftLeader.setVoltage(volts.in(Volts));
        rightLeader.setVoltage(volts.in(Volts));
    }

    public void log(SysIdRoutineLog log) {
        int numberOfEntries = 2;
        double averageVoltage = ((leftLeader.getAppliedOutput() * leftLeader.getBusVoltage()) +
                (rightLeader.getAppliedOutput() * rightLeader.getBusVoltage())) / numberOfEntries;

        double averageLinearPosition = (getLefEncoderPosition() + getRightEncoderPosition()) / numberOfEntries;

        double averageLinearVelocity = (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / numberOfEntries;

        // drivetrain
        log.motor("drivetrain")
                .voltage(appliedVoltage.mut_replace(averageVoltage, Volts))
                .linearPosition(distance.mut_replace(averageLinearPosition, Meters))
                .linearVelocity(velocity.mut_replace(averageLinearVelocity, MetersPerSecond));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return driveTrainRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return driveTrainRoutine.dynamic(direction);
    }

    public void arcadeDrive(double rotate, double drive) {
        // variables for determining the quadrants
        double maximum = Math.max(Math.abs(drive), Math.abs(rotate));
        double total = drive + rotate;
        double difference = drive - rotate;

        // set speed according to the quadrant that the values are in
        if (drive >= 0) {
            if (rotate >= 0) { // quadrant 1
                leftLeader.set(maximum);
                rightLeader.set(difference);
            } else { // quadrant 2
                leftLeader.set(total);
                rightLeader.set(maximum);
            }
        } else {
            if (rotate >= 0) { // quadrant 4
                leftLeader.set(total);
                rightLeader.set(-maximum);
            } else { // quadrant 3
                leftLeader.set(-maximum);
                rightLeader.set(difference);
            }
        }
    }

    // algorithm to apply curvature drive to our arcade drive
    // thanks to this guide:
    // https://compendium.readthedocs.io/en/latest/tasks/drivetrains/advancedtank.html
    public double applyCurve(double joystickPosition) {
        // first part of equation is the same so extract to variable.
        final double pt1 = (1 - Constants.TORQUE_RESISTANCE_THRESHOLD) * Math.pow(joystickPosition, 3);

        // piecewise logic
        if (joystickPosition > 0) {
            return pt1 + Constants.TORQUE_RESISTANCE_THRESHOLD;
        } else if (joystickPosition < 0) {
            return pt1 - Constants.TORQUE_RESISTANCE_THRESHOLD;
        }

        // else joystick position is 0 so return 0.
        return 0;
    }

    // algorithm for handling joystick drift by a deadzone implementation
    public double deadZone(double value, double deadZone) {
        if (Math.abs(value) < deadZone) {
            return 0;
        }
        return value;
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public double getLefEncoderPosition() {
        return -leftEncoder.getPosition();
    }

    public double getRightEncoderPosition() {
        return -rightEncoder.getPosition();
    }

    public double getHeading() {
        return navX.getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        return -navX.getRate();
    }

    public Pose2d getPose2d() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(),
                new Pose2d());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());

    }

    @Override
    public void periodic() {
        odometry.update(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
        SmartDashboard.putNumber("Gyro Heading", getHeading());

        SmartDashboard.putNumber("Left Encoder Value (feet)", -leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Encoder Value (feet) ", -rightEncoder.getPosition());
    }
}
