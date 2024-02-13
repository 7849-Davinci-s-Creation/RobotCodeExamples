// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.DriveTrain;

public final class Autos {
  private static SendableChooser<Command> autoMenu = null;

  private Autos() {

  }

  public static SendableChooser<Command> getAutoMenu() {
    if (autoMenu == null) {
      autoMenu = new SendableChooser<>();
    }
    return autoMenu;
  }

  public static Command driveSysIDQuasistaticForward(DriveTrain driveTrain) {
    return driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward);
  }

  public static Command driveSysIDQuasistaticBackwards(DriveTrain driveTrain) {
    return driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse);
  }

  public static Command driveSysIDDynamicForwards(DriveTrain driveTrain) {
    return driveTrain.sysIdDynamic(SysIdRoutine.Direction.kForward);
  }

  public static Command driveSysIDDynamicBackwards(DriveTrain driveTrain) {
    return driveTrain.sysIdDynamic(SysIdRoutine.Direction.kReverse);
  }
}
