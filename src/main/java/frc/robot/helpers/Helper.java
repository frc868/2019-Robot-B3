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
        return boundValue(value, -1, 1);
    }

    /**
     * Creates a virtual deadzone. 
     * If a value is between the lower max and upper min, it will output zero. 
     * If a value is outside this range, the original value will be outputted.
     * @param value value to deadzone
     * @param size the size of the deadzone
     * @return the "deadzoned" version of the input value
     */
    public static double deadzone(double value, double size) {
        if (Math.abs(value) < Math.abs(size)) {
            return 0;
        } else {
            return value;
        }
    }
}