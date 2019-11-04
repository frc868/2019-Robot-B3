package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.helpers.ControllerWrapper;

/**
 * The class in which we map our driver/operator input to specific tasks on the robot
 * Init should be called once in the robotInit() method in the Robot class
 * Update should be called either in robotPeriodic() or teleopPeriodic()
 * 
 * @author dri
 */
public class OI {
    public static ControllerWrapper driver = new ControllerWrapper(RobotMap.Controllers.DRIVER_PORT);
    public static ControllerWrapper operator = new ControllerWrapper(RobotMap.Controllers.OPERATOR_PORT);

    public static void init() {
        Robot.manipulator.grab();
    }

    public static void update() {
        // GENERAL CONTROLS/CONTROL METHODS
        Robot.drivetrain.arcadeDrive();
        Robot.manipulator.manualIntake();

        // DRIVER CONTROLS
        driver.bA.whileHeld(() -> Robot.camera.manualFollowVision());

        // TODO: B -- TurnToAngleGyro(130)
        // TODO: X -- TurnToAngleGyro(-130)
        // TODO: POV W -- TurnToAngleGyro(-90)
        // TODO: POV E -- TurnToAngleGyro(90)

        driver.bLB.whenPressed(() -> Robot.manipulator.toggle());

        driver.bMENU.whenPressed(() -> {
            Robot.climberArms.deploy();
            // TODO: Set tilt position "Upper"
            Robot.manipulator.grab();
        });
        // TODO: Start -- ManualClimber

        // OPERATOR CONTROLS
        operator.bLB.whenPressed(() -> Robot.manipulator.toggle());

        Robot.climberElevator.setSpeed(operator.getLY());

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