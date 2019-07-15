package frc.robot;

import edu.wpi.first.wpilibj.XboxController;


/**
 * The class in which we map our driver input to specific tasks on the robot
 * Init should be called once in the robotInit() method in the Robot class
 * Update should be called either in robotPeriodic() or teleopPeriodic()
 * 
 * @author DRI
 */
public class OI{
    public static XboxController driver = new XboxController(RobotMap.Controllers.DRIVER_PORT);
    public static XboxController operator = new XboxController(RobotMap.Controllers.OPERATOR_PORT);

    public static void init() {
        Robot.hatchClaw.grab();
    }

    public static void update() {
        double y = driver.getRawAxis(RobotMap.Controllers.LY);
        double x = driver.getRawAxis(RobotMap.Controllers.RX);
        Robot.drivetrain.arcadeDrive(y, x);
    }
}