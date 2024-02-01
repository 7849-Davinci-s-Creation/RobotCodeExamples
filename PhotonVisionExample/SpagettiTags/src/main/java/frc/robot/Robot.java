// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  PS4Controller ps4Controller;

  MotorController rightMotors;
  MotorController leftMotors;

  DifferentialDrive driveType;
  // Constants such as camera and target height stored. Change per robot and goal!

  final double CAMERA_HEIGHT_METERS = 0.3;

  final double TARGET_HEIGHT_METERS = 0.66;

  // Angle between horizontal and the camera.

  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(23);
  final double GOAL_RANGE_METERS = 4;

  // Change this to match the name of your camera

  PhotonCamera camera = new PhotonCamera("CoolCam");

  // PID constants should be tuned per robot

  final double LINEAR_P = 1;

  final double LINEAR_D = 0.05;

  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.023;

  final double ANGULAR_D = 0.011;

  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  private WPI_VictorSPX frontRightMotor;

  private WPI_VictorSPX frontLeftMotor;

  private WPI_VictorSPX backLeftMotor;

  private WPI_VictorSPX backRightMotor;

  public Robot() {
    frontRightMotor = new WPI_VictorSPX(3);
    frontLeftMotor = new WPI_VictorSPX(4);
    backLeftMotor = new WPI_VictorSPX(1);
    backRightMotor = new WPI_VictorSPX(2);

    ps4Controller = new PS4Controller(0);

    rightMotors = new MotorControllerGroup(frontRightMotor, backRightMotor);
    leftMotors = new MotorControllerGroup(frontLeftMotor, backRightMotor);
    driveType = new DifferentialDrive(rightMotors, leftMotors);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double moveSpeed;
    double rotateSpeed;

    if (ps4Controller.getCrossButton()) {
      // Vision-alignment mode
      // Query the latest result from PhotonVision
      var result = camera.getLatestResult();

      if (result.hasTargets()) {
        // Calculate angular turn power
        // -1.0 required to ensure positive PID controller effort _increases_ yaw
        double range = PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(result.getBestTarget().getPitch()));

        moveSpeed = forwardController.calculate(range, GOAL_RANGE_METERS);
        rotateSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
      } else {
        // If we have no targets, stay still.
        rotateSpeed = 0;
        moveSpeed = 0;
      }

    } else {
      rotateSpeed = ps4Controller.getLeftY();
      moveSpeed = ps4Controller.getRightX();
    }

    driveType.arcadeDrive(rotateSpeed, moveSpeed);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
