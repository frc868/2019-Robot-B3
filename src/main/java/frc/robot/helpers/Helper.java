package frc.robot.helpers;

public class Helper{

    public static final double DEADZONE_RANGE = 0.3;

    /**
     * 
     * @param value the input (probably joystick)
     * @param range the limit of the deadband you want to apply
     * @return 0 if input is in deadzone, else return value (no change)
     */
    public static double deadband(double value, double range){
        return Math.abs(value) < range ? 0 : value;
    }

}