package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public  class MoveForwardSeconds extends CommandBase {
    private DriveTrain driveTrain;
    private double seconds;
    private double power;

    private double startingtime;

    public MoveForwardSeconds(DriveTrain driveTrain, double seconds, double power) {
       addRequirements(driveTrain);
       this.driveTrain = driveTrain;
       this.seconds = seconds;
       this.power = power;  
    }

    @Override
    public void initialize() {
      startingtime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        driveTrain.forward(power);
    }

    @Override
    public void end(boolean interuppted) {
        driveTrain.forward(0);
    }

    @Override 
    public boolean isFinished() {
      return System.currentTimeMillis() - startingtime > (seconds * 1000);
    }
}
