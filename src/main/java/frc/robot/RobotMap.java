package frc.robot;

public class RobotMap {
    public static int PCM = 0;

    public static class Drivetrain {
        public static final int LEFT_PRIMARY = 23; // untested
        public static final int LEFT_SECONDARY = 24; // untested
        public static final int LEFT_TERTIARY = 25; // untested

        public static final int RIGHT_PRIMARY = 10; // untested
        public static final int RIGHT_SECONDARY = 11; // untested
        public static final int RIGHT_TERTIARY = 12; // untested
    }

    public static class ClimberElevator {
        public static final int PRIMARY = 13;
        public static final int SECONDARY = 22;
        public static final boolean PRIMARY_IS_INVERTED = false; // untested
        public static final boolean SECONDARY_IS_INVERTED = true; // untested

        public static final int SWITCHER = 2;    // untested
        public static final int ELEV_BRAKE = 3;  // untested
        public static final int CLIMB_BRAKE = 4; // untested

        public static final int ELEV_TOP_LIM = 0; // untested
        public static final int ELEV_BOT_LIM = 1; // untested
        public static final int CLIMB_LIM_SWITCH = 4; // untested
    }

    public static class Sensors {
        public static final int GYRO = 1; // untested
    }

    public static class Controllers {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;

        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int RB = 6;
        public static final int LB = 5;
        public static final int RSTK = 10;
        public static final int LSTK = 9;
        public static final int START = 8;
        public static final int MENU = 7;

        public static final int LX = 0;
        public static final int LY = 1; // Arcade drive
        public static final int RX = 4; // Arcade drive
        public static final int RY = 5;
        public static final int LT = 2;
        public static final int RT = 3;
    }

    public static class ClimberArms {
        public static final int SOLENOID_CHANNEL = 0;
    }
}