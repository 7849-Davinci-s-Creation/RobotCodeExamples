package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class MoveFeet extends CommandBase {
    private final DriveTrain driveTrain;
    private final double feet;
    private final double kP = 0.0811;
    // do not set this to 100 or i will steal your milkshake
    private final double kI = 0.0002;
    private final double iLimit = 1;
    double errorSum = 0;
    double lastTimestamp = 0;

    public MoveFeet(DriveTrain driveTrain, double feet) {
        this.driveTrain = driveTrain;
        this.feet = feet;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        errorSum = 0;
        lastTimestamp = Timer.getFPGATimestamp(); 
          
        
    }

    @Override
    public void execute() {
        double error = feet - driveTrain.ticks2Feet(driveTrain.getLefEncoderPosition());
        double dt = Timer.getFPGATimestamp() - lastTimestamp;
        if (Math.abs(error)< iLimit) {
        errorSum += error * dt;
        }
        double outputSpeed = kP * error + kI * errorSum;
        driveTrain.arcadeDrive(-outputSpeed,0);
        lastTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interuppted) {
        driveTrain.arcadeDrive(0, 0);
        System.out.println("End of auto rotations\nleft: " + driveTrain.getLefEncoderPosition() + " right: " + driveTrain.getRightEncoderPosition());
    }

    @Override
    public boolean isFinished() {
        return driveTrain.ticks2Feet(driveTrain.getLefEncoderPosition()) >= feet 
        && 
        driveTrain.ticks2Feet(driveTrain.getRightEncoderPosition()) >= feet;
    }
    
}
