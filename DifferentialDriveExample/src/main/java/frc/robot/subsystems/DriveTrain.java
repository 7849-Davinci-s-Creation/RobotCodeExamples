package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
    private final CANSparkMax leftLeader = new CANSparkMax(Constants.FRONTLEFTMOTOR_PORT,MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(Constants.BACKLEFTMOTOR_PORT,MotorType.kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(Constants.FRONTRIGHTMOTOR_PORT,MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(Constants.BACKRIGHTMOTOR_PORT,MotorType.kBrushless);
    
    private final MotorControllerGroup leftGroup = new MotorControllerGroup(leftLeader, leftFollower);
    private final MotorController rightGroup = new MotorControllerGroup(rightLeader, rightFollower);

    private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
    private final RelativeEncoder rightEncoder = leftFollower.getEncoder();

    public DriveTrain() {
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);
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
        // variables for determening the quadrants
        double maximum = Math.max(Math.abs(drive), Math.abs(rotate));
        double total = drive + rotate;
        double difference = drive - rotate;

        // set speed according to the quadrant that the values are in
        if (drive >= 0) {
            if (rotate >= 0) { // quadrant 1 
                leftGroup.set(maximum);
                rightGroup.set(difference);
            } else { // quadrant 2
                leftGroup.set(total);
                rightGroup.set(maximum);
            }
        } else { 
            if (rotate >= 0) { // quadrant 4
                leftGroup.set(total);
                rightGroup.set(-maximum);
            } else { // quadrant 3
                leftGroup.set(-maximum);
                rightGroup.set(difference);
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
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Front Left Motor Current", leftLeader.getOutputCurrent());
        SmartDashboard.putNumber("Back Left Motor Current", leftFollower.getOutputCurrent());
        SmartDashboard.putNumber("Front Right Motor Current", rightLeader.getOutputCurrent());
        SmartDashboard.putNumber("Back Right Motor Current", rightFollower.getOutputCurrent());

        SmartDashboard.putNumber("Left Encoder Value (feet)", ticks2Feet(-leftEncoder.getPosition()));
        SmartDashboard.putNumber("Right Encoder Value (feet) ", ticks2Feet(-rightEncoder.getPosition()));

        SmartDashboard.putNumber("Left Encoder Rotations", getLefEncoderPosition());
        SmartDashboard.putNumber("Right Encoder Rotations", getLefEncoderPosition());

    }

    public void teleOpPeriodic() {
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

}
