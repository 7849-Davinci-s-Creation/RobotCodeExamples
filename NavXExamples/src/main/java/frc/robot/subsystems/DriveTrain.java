package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
    private AHRS navx;

    private WPI_VictorSPX frontRightMotor;
    private WPI_VictorSPX frontLeftMotor;
    private WPI_VictorSPX backLeftMotor;
    private WPI_VictorSPX backRightMotor;

    public DriveTrain() {
        frontRightMotor = new WPI_VictorSPX(3);
        frontLeftMotor = new WPI_VictorSPX(4);
        backLeftMotor = new WPI_VictorSPX(1);
        backRightMotor = new WPI_VictorSPX(2);

        backLeftMotor.follow(frontLeftMotor);
        backRightMotor.follow(frontRightMotor);

        try {
            /* Communicate w/navX-MXP via the MXP SPI Bus. */
            /* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
            /*
             * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
             * details.
             */
            navx = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }

    }

    public void arcadeDrive(double rotate, double drive) {
        double maximum = Math.max(Math.abs(rotate), Math.abs(drive));
        double total = drive + rotate;
        double difference = drive - rotate;

        if (drive >= 0) {
            if (rotate >= 0) {
                frontLeftMotor.set(maximum);
                frontRightMotor.set(difference);

            } else {
                frontLeftMotor.set(total);
                frontRightMotor.set(maximum);
            }
        } else {
            if (rotate >= 0) {
                frontLeftMotor.set(total);
                frontRightMotor.set(-maximum);
            } else {
                frontLeftMotor.set(-maximum);
                frontRightMotor.set(difference);
            }
        }

    }

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

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("IMU_Connected", navx.isConnected());
        SmartDashboard.putBoolean("IMU_IsCalibrating", navx.isCalibrating());
        SmartDashboard.putNumber("IMU_Yaw", navx.getYaw());
        SmartDashboard.putNumber("IMU_Pitch", navx.getPitch());
        SmartDashboard.putNumber("IMU_Roll", navx.getRoll());

        /* Display tilt-corrected, Magnetometer-based heading (requires */
        /* magnetometer calibration to be useful) */

        SmartDashboard.putNumber("IMU_CompassHeading", navx.getCompassHeading());

        /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
        SmartDashboard.putNumber("IMU_FusedHeading", navx.getFusedHeading());

        /* These functions are compatible w/the WPI Gyro Class, providing a simple */
        /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP */

        SmartDashboard.putNumber("IMU_TotalYaw", navx.getAngle());
        SmartDashboard.putNumber("IMU_YawRateDPS", navx.getRate());

        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

        SmartDashboard.putNumber("IMU_Accel_X", navx.getWorldLinearAccelX());
        SmartDashboard.putNumber("IMU_Accel_Y", navx.getWorldLinearAccelY());
        SmartDashboard.putBoolean("IMU_IsMoving", navx.isMoving());
        SmartDashboard.putBoolean("IMU_IsRotating", navx.isRotating());

        /* Display estimates of velocity/displacement. Note that these values are */
        /* not expected to be accurate enough for estimating robot position on a */
        /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
        /* of these errors due to single (velocity) integration and especially */
        /* double (displacement) integration. */

        SmartDashboard.putNumber("Velocity_X", navx.getVelocityX());
        SmartDashboard.putNumber("Velocity_Y", navx.getVelocityY());
        SmartDashboard.putNumber("Displacement_X", navx.getDisplacementX());
        SmartDashboard.putNumber("Displacement_Y", navx.getDisplacementY());

        /* Display Raw Gyro/Accelerometer/Magnetometer Values */
        /* NOTE: These values are not normally necessary, but are made available */
        /* for advanced users. Before using this data, please consider whether */
        /* the processed data (see above) will suit your needs. */

        SmartDashboard.putNumber("RawGyro_X", navx.getRawGyroX());
        SmartDashboard.putNumber("RawGyro_Y", navx.getRawGyroY());
        SmartDashboard.putNumber("RawGyro_Z", navx.getRawGyroZ());
        SmartDashboard.putNumber("RawAccel_X", navx.getRawAccelX());
        SmartDashboard.putNumber("RawAccel_Y", navx.getRawAccelY());
        SmartDashboard.putNumber("RawAccel_Z", navx.getRawAccelZ());
        SmartDashboard.putNumber("RawMag_X", navx.getRawMagX());
        SmartDashboard.putNumber("RawMag_Y", navx.getRawMagY());
        SmartDashboard.putNumber("RawMag_Z", navx.getRawMagZ());
        SmartDashboard.putNumber("IMU_Temp_C", navx.getTempC());

        /* Omnimount Yaw Axis Information */
        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
        AHRS.BoardYawAxis yaw_axis = navx.getBoardYawAxis();
        SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
        SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());

        /* Sensor Board Information */
        SmartDashboard.putString("FirmwareVersion", navx.getFirmwareVersion());

        /* Quaternion Data */
        /* Quaternions are fascinating, and are the most compact representation of */
        /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
        /* from the Quaternions. If interested in motion processing, knowledge of */
        /* Quaternions is highly recommended. */
        SmartDashboard.putNumber("QuaternionW", navx.getQuaternionW());
        SmartDashboard.putNumber("QuaternionX", navx.getQuaternionX());
        SmartDashboard.putNumber("QuaternionY", navx.getQuaternionY());
        SmartDashboard.putNumber("QuaternionZ", navx.getQuaternionZ());

        /* Connectivity Debugging Support */
        SmartDashboard.putNumber("IMU_Byte_Count", navx.getByteCount());
        SmartDashboard.putNumber("IMU_Update_Count", navx.getUpdateCount());
    }

}
