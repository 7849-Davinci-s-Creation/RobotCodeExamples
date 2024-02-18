// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {
  private final DriveTrain driveTrain = new DriveTrain();
  private final CommandPS4Controller controller = new CommandPS4Controller(Constants.Controllers.JOYSTICK_PORT);

  private final SendableChooser<Command> autoMenu = Autos.getAutoMenu();

  public RobotContainer() {
    configureBindings();
    configureAutoMenu();
    driveTrain.resetEncoders();
  }

  private void configureBindings() {
    driveTrain.setDefaultCommand(new Drive(controller, driveTrain));
  }

  private void configureAutoMenu() {
    autoMenu.addOption(
            "Drive SysId (Quasistatic Forward)", Autos.driveSysIDQuasistaticForward(driveTrain));
    autoMenu.addOption(
            "Drive SysId (Quasistatic Reverse)", Autos.driveSysIDQuasistaticBackwards(driveTrain));
    autoMenu.addOption(
            "Drive SysId (Dynamic Forward)", Autos.driveSysIDDynamicForwards(driveTrain));
    autoMenu.addOption(
            "Drive SysId (Dynamic Reverse)", Autos.driveSysIDDynamicBackwards(driveTrain));

    SmartDashboard.putData(autoMenu);
  }

  public Command getAutonomousCommand() {
    return autoMenu.getSelected();
  }

  public void autonomousInit() {
    driveTrain.resetMotors();
  }
}
