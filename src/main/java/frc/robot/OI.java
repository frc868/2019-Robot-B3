package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.TurnToAngleGyro;
import frc.robot.helpers.ControllerWrapper;
import frc.robot.subsystems.ClimberElevator.ElevatorPosition;
import frc.robot.subsystems.Tilt.TiltPosition;

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

    public static TurnToAngleGyro turnTo = TurnToAngleGyro.getInstance();

    public static void init() {
        Robot.manipulator.grab();
    }

    public static void update() {
        // GENERAL CONTROLS/CONTROL METHODS
        Robot.drivetrain.arcadeDrive(0.85); // training mode
        Robot.manipulator.manualIntake();

        // DRIVER CONTROLS
        driver.bA.whileHeld(() -> Robot.camera.manualFollowVision());
        driver.bA.whenReleased(() -> Robot.drivetrain.setSpeed(0, 0)); // cancel manualFollowVision

        driver.bB.whenPressed(() -> turnTo.run(130));
        driver.bB.whenReleased(() -> turnTo.stop());
        driver.bX.whenPressed(() -> turnTo.run(-130));
        driver.bX.whenReleased(() -> turnTo.stop());
        driver.dW.whenPressed(() -> turnTo.run(-90));
        driver.dW.whenReleased(() -> turnTo.stop());
        driver.dE.whenPressed(() -> turnTo.run(90));
        driver.dE.whenReleased(() -> turnTo.stop());

        driver.bLB.whenPressed(() -> Robot.manipulator.toggle());

        driver.bMENU.whenPressed(() -> {
            Robot.climberArms.deploy();
            // TODO: Set tilt position "Upper"
            Robot.manipulator.grab();
        });
        // TODO: Start -- ManualClimber

        // OPERATOR CONTROLS
        operator.bY.whenPressed(() -> Robot.climberElevator.setPosition(ElevatorPosition.HIGH));
        operator.bB.whenPressed(() -> Robot.climberElevator.setPosition(ElevatorPosition.MIDDLE));
        operator.bA.whenPressed(() -> Robot.climberElevator.setPosition(ElevatorPosition.LOW));

        operator.dN.whenPressed(() -> Robot.tilt.setPosition(TiltPosition.HIGH));
        operator.dE.whenPressed(() -> Robot.tilt.setPosition(TiltPosition.MIDDLE));
        operator.dS.whenPressed(() -> Robot.tilt.setPosition(TiltPosition.LOW));

        operator.bLB.whenPressed(() -> Robot.manipulator.toggle());

        updateSD();
    }

    /**
     * Updates the SmartDashboard with values of sensors
     */
    public static void updateSD() {
        SmartDashboard.putBoolean("Climber Top Limit", Robot.climberElevator.getTopLimit());
        SmartDashboard.putBoolean("Climber Bot Limit", Robot.climberElevator.getBotLimit());
        SmartDashboard.putNumber("Elevator Power", Robot.climberElevator.getElevatorSpeed());
    }
}