package frc.robot;

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

    public static final double TORQUE_RESISTANCE_THRESHOLD = 0.05F;
    public static final double JOYSTICK_DEADZONE_DRIVE = 0.01F;
    public static final double JOYSTICK_DEADZONE_ROTATE = 0.01F;

    public static final double MOVEMENT_NERF = 1.5F;    

}
