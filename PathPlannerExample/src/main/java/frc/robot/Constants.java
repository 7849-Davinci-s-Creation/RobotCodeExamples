package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final int FRONTLEFTMOTOR_PORT = 2;
    public static final int FRONTRIGHTMOTOR_PORT = 9;
    public static final int BACKRIGHTMOTOR_PORT = 6;
    public static final int BACKLEFTMOTOR_PORT = 5;
    
    public static final class Controllers {
        public static final int JOYSTICK_PORT = 0;
        public static final int JOYSTICKY = 1;
        public static final int JOYSTICKZ = 2;
    }
    public static final class SysIDvalues {
        public static final double KS = 0.18533;
        public static final double KV = 145.98;
        public static final double KA = 31.967;
        public static final double KP = 15.455; 
    }
    public static final double kTrackWidthMeters = 0.51435;
    public static final DifferentialDriveKinematics drivekinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
    public static final double kMaxSpeedMPS = 3;
    public static final double kMaxAccelerationMPSsq = 3;
    public static final double RamseteB = 2;
    public static final double RamsetZ = 0.7;
    public static final double gearRatio = 5.95;
    public static final double wheelRadiusInches = 3;

    public static final double kLinearConversion = Units.inchesToMeters(
        1/(gearRatio * 2 * Math.PI * Units.inchesToMeters(wheelRadiusInches)) * 10)
        ;
    

    public static final double TORQUE_RESISTANCE_THRESHOLD = 0.05F;
    public static final double JOYSTICK_DEADZONE_DRIVE = 0.01F;
    public static final double JOYSTICK_DEADZONE_ROTATE = 0.01F;

    public static final double MOVEMENT_NERF = 1.5F;    

}

