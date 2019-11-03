package frc.robot.helpers;

public class Helper {
    /**
     * Given a value, checks to makes sure it is between min and max. If not replace with the value
     *  of min or max. If so return value
     * @param value The value to check
     * @param min The minimum acceptable value
     * @param max The maximum acceptable value
     * @return The sanity value after check
     * @author lm
     */
    public static double boundValue(double value, double min, double max) {
        if (value > max) {
            return max;
        }
        if (value < min) {
            return min;
        }
        return value;
    }
    /**
     * Given a value, checks to makes sure it is between -1.0 and 1.0. If not replace with the 
     * value with -1.0 or 1.0. If so return value
     * @param value The value to check
     * @return The sanity value after check
     * @author lm
     */
    public static double boundValue(double value) {
        if (value > 1.0) {
            return 1.0;
        }
        if (value < -1.0) {
            return -1.0;
        }
        return value; 
    }
}