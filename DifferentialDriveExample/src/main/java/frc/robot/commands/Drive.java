package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class Drive extends CommandBase {
    private final CommandXboxController controller;
    private final DriveTrain driveTrain;

    public Drive(CommandXboxController controller, DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.controller = controller;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        // need to test this new driving code.
        double rotate = driveTrain.deadZone(controller.getRawAxis(Constants.Controllers.JOYSTICKY), Constants.JOYSTICK_DEADZONE_ROTATE);
        double forwardBack =  driveTrain.deadZone(controller.getRawAxis(Constants.Controllers.JOYSTICKZ), Constants.JOYSTICK_DEADZONE_DRIVE);

        driveTrain.arcadeDrive(driveTrain.applyCurve(rotate / Constants.MOVEMENT_NERF), driveTrain.applyCurve(forwardBack));

        // to test out other driving method comment out above and uncomment below
        // driveTrain.semiConstantCurvatureDrive(forwardBack, rotate);
    }

    @Override
    public void end(boolean interuppted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
