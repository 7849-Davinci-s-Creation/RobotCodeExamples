package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class DriveTrain extends SubsystemBase {
    private final CANSparkMax leftLeader = new CANSparkMax(Constants.FRONTLEFTMOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(Constants.BACKLEFTMOTOR_PORT,MotorType.kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(Constants.FRONTRIGHTMOTOR_PORT,MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(Constants.BACKRIGHTMOTOR_PORT,MotorType.kBrushless);

    private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
    private final RelativeEncoder rightEncoder = leftFollower.getEncoder();

    // Create the SysId routine
    SysIdRoutine driveTrainRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.1).per(Seconds.of(1)), Volts.of(7), Seconds.of(10), // Use default config (NOTE: EDITED TO NOT BE SO FAST)
                    (state) -> Logger.recordOutput("SysIdTestState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    (voltage) -> this.voltageDrive(voltage.in(Volts), voltage.in(Volts)),
                    null, // No log consumer, since data is recorded by AdvantageKit
                    this
            )
    );

    public DriveTrain() {
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);
        leftLeader.setInverted(true);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return driveTrainRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return driveTrainRoutine.dynamic(direction);
    }

    public void voltageDrive(double left, double right) {
        leftLeader.set(left);
        rightLeader.set(right);
    }

    /*
     * Think of a X,Y plane (What you would use to create a graph).
     * We can actually split each section of the graph into quadrants.
     * -----------------
     * |      |        |
     * |  2   |   1    |    We do that like so !
     * |      |        |    So if I have a positive pair of numbers (we call those coordinates) like (1,3) for example, 
     * |------|--------|    the point we would place would be in quadrant 1 and so forth depending on the points you put on the graph.
     * |   3  |   4    |
     * |      |        |
     * -----------------
     */
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

    public double ticks2Feet(double encoderPosition) {
        return encoderPosition * ((6*Math.PI)/71.4);
    }

    // algorithm to apply curvature drive to our arcade drive
    // thanks to this guide: https://compendium.readthedocs.io/en/latest/tasks/drivetrains/advancedtank.html
    public double applyCurve(double joystickPosition) {
        // first part of equation is the same so extract to variable.
        final double pt1 = (1 - Constants.TORQUE_RESISTANCE_THRESHOLD) * Math.pow(joystickPosition, 3);

        // piecewise logic
        if (joystickPosition > 0) {
            return  pt1 + Constants.TORQUE_RESISTANCE_THRESHOLD;
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
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Front Left Motor Current", leftLeader.getOutputCurrent());
        SmartDashboard.putNumber("Back Left Motor Current", leftFollower.getOutputCurrent());
        SmartDashboard.putNumber("Front Right Motor Current", rightLeader.getOutputCurrent());
        SmartDashboard.putNumber("Back Right Motor Current", rightFollower.getOutputCurrent());

        SmartDashboard.putNumber("Left Encoder Value (feet)", ticks2Feet(-leftEncoder.getPosition()));
        SmartDashboard.putNumber("Right Encoder Value (feet) ", ticks2Feet(-rightEncoder.getPosition()));
    }
}
