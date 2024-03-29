package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.TurnToAngleGyro;
import frc.robot.helpers.ControllerWrapper;
import frc.robot.helpers.Helper;
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
    public static ControllerWrapper driver = new ControllerWrapper(RobotMap.Controllers.DRIVER_PORT, true);
    public static ControllerWrapper operator = new ControllerWrapper(RobotMap.Controllers.OPERATOR_PORT, true);

    public static TurnToAngleGyro turnTo = TurnToAngleGyro.getInstance();

    public static void init() {
        Robot.manipulator.grab();
    }

    public static void update() {
        // GENERAL CONTROLS/CONTROL METHODS
        if (Robot.climberElevator.isElevatorMode()) { // only in elevator/drive mode
            Robot.drivetrain.arcadeDrive(0.85); // training mode
            Robot.climberElevator.manualElevator(0.5); // training mode
        } else { // only in climb mode
            Robot.drivetrain.arcadeDrive(0.25); // limit drivetrain speed to prevent tipping
        }
        Robot.manipulator.manualIntake();

        // DRIVER CONTROLS
        driver.bA.whileHeld(() -> Robot.camera.manualFollowVision());
        driver.bA.whenReleased(() -> Robot.drivetrain.setSpeed(0, 0)); // cancel manualFollowVision

        // uncomment these when we're confident that Indu won't kill Fronek
        // also they're kinda useless
        /* driver.bB.whenPressed(() -> turnTo.run(130));
        driver.bB.whenReleased(() -> turnTo.stop());
        driver.bX.whenPressed(() -> turnTo.run(-130));
        driver.bX.whenReleased(() -> turnTo.stop());
        driver.dW.whenPressed(() -> turnTo.run(-90));
        driver.dW.whenReleased(() -> turnTo.stop());
        driver.dE.whenPressed(() -> turnTo.run(90));
        driver.dE.whenReleased(() -> turnTo.stop()); */

        driver.bLB.whenPressed(() -> Robot.manipulator.toggle());

        // releases sled runners and puts robot in climb position
        driver.bMENU.whenPressed(() -> {
            Robot.climberArms.deploy();
            Robot.tilt.setPosition(TiltPosition.HIGH);
            Robot.manipulator.grab();
        });
        driver.bSTART.whenPressed(() -> Robot.climberElevator.switchToggle());
        // TODO: don't know if ^ is a good idea

        // OPERATOR CONTROLS
        operator.bLB.whenPressed(() -> Robot.manipulator.toggle());

        operator.dN.whenPressed(() -> Robot.tilt.setPosition(TiltPosition.HIGH));
        operator.dE.whenPressed(() -> Robot.tilt.setPosition(TiltPosition.MIDDLE));
        operator.dS.whenPressed(() -> Robot.tilt.setPosition(TiltPosition.LOW));

        if (Robot.climberElevator.isElevatorMode()) { // elevator mode only
            operator.bY.whenPressed(() -> Robot.climberElevator.setPosition(ElevatorPosition.HIGH));
            operator.bB.whenPressed(() -> Robot.climberElevator.setPosition(ElevatorPosition.MIDDLE));
            operator.bA.whenPressed(() -> Robot.climberElevator.setPosition(ElevatorPosition.LOW));
        } else { // climb mode only
            Robot.climberElevator.setSpeed(Helper.boundValue(operator.getLY())); // obsoletes ManualElevator!
            Robot.climberElevator.driveFoot(Helper.boundValue(operator.getRY(), -1, 0)); // foot drive on RY
            // TODO: ^ may need to be inverted
        }

        updateSD();
    }

    /**
     * Updates the SmartDashboard with values of sensors
     */
    public static void updateSD() {
        SmartDashboard.putBoolean("Climber Top Limit", Robot.climberElevator.getTopLimit());
        SmartDashboard.putBoolean("Climber Bot Limit", Robot.climberElevator.getBotLimit());
        SmartDashboard.putNumber("Elevator Power", Robot.climberElevator.getElevatorSpeed());
        SmartDashboard.putNumber("Tilt Position", Robot.tilt.getPotPosition());
    }
}