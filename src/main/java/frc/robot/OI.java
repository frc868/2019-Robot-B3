package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The class in which we map our driver/operator input to specific tasks on the robot
 * Init should be called once in the robotInit() method in the Robot class
 * Update should be called either in robotPeriodic() or teleopPeriodic()
 * 
 * @author dri
 */
public class OI{
    public static XboxController driver = new XboxController(RobotMap.Controllers.DRIVER_PORT);
    public static XboxController operator = new XboxController(RobotMap.Controllers.OPERATOR_PORT);

    public static void init() {
        Robot.hatchClaw.grab();
    }

    public static void update() {
        Robot.drivetrain.arcadeDrive();
        if(operator.getRawButtonPressed(RobotMap.Controllers.LB)) {
            Robot.hatchClaw.toggle();
        }
        Robot.climberElevator.setSpeed(operator.getRawAxis(RobotMap.Controllers.LY));

        updateSD();
    }

    /**
     * Updates the SmartDashboard with values of sensors
     */
    public static void updateSD() {
        SmartDashboard.putBoolean("Climber Top Limit", Robot.climberElevator.getTopLimit());
        SmartDashboard.putBoolean("Climber Bot Limit", Robot.climberElevator.getBotLimit());


    }
}