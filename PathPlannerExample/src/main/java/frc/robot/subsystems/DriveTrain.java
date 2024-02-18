package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import lib.Encoderutil;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class DriveTrain extends SubsystemBase {
    private final CANSparkMax leftLeader = new CANSparkMax(Constants.FRONTLEFTMOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(Constants.BACKLEFTMOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(Constants.FRONTRIGHTMOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(Constants.BACKRIGHTMOTOR_PORT, MotorType.kBrushless);

    private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
    private final RelativeEncoder rightEncoder = leftFollower.getEncoder();

    SysIdRoutine.Config config = new SysIdRoutine.Config(Volts.of(3).per(Seconds.of(1)),
            Volts.of(3),
            Seconds.of(3),
            state -> Logger.recordOutput("SysIdTestState", state.toString())
    );

    SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
            this::voltageDrive,
            null,
            this);

    SysIdRoutine driveTrainRoutine = new SysIdRoutine(config, mechanism);

    public DriveTrain() {
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);
        leftLeader.setInverted(true);

        leftLeader.setIdleMode(IdleMode.kBrake);
        rightLeader.setIdleMode(IdleMode.kBrake);
        leftFollower.setIdleMode(IdleMode.kBrake);
        rightFollower.setIdleMode(IdleMode.kBrake);

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

    public void resetMotors() {
        this.voltageDrive(Volts.of(0));
    }

    public double getLefEncoderPosition() {
        return -leftEncoder.getPosition();
    }

    public double getRightEncoderPosition() {
        return -rightEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Front Left Motor Current", leftLeader.getOutputCurrent());
        SmartDashboard.putNumber("Back Left Motor Current", leftFollower.getOutputCurrent());
        SmartDashboard.putNumber("Front Right Motor Current", rightLeader.getOutputCurrent());
        SmartDashboard.putNumber("Back Right Motor Current", rightFollower.getOutputCurrent());

        SmartDashboard.putNumber("Left Encoder Value (feet)", -leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Encoder Value (feet) ", -rightEncoder.getPosition());
    }
}
