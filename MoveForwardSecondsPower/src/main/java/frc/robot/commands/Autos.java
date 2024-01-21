// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;

public final class Autos {

  // Create an autonomous command for running the robot forward for the passed seconds and power
  public static MoveForwardSeconds moveForward(DriveTrain driveTrain, double seconds, double power) {
    return new MoveForwardSeconds(driveTrain, seconds, power);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
