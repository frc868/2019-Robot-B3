/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.OI;
import frc.robot.RobotMap;

/**
 * This class is the robot's Drivetrain. It is a singleton to avoid instantiating it multiple times
 * @author dri
 */
public class Drivetrain {
    // These are the motor controllers which we are using
    private CANSparkMax l_primary, l_secondary, l_tertiary, r_primary, r_secondary, r_tertiary;
    private SpeedControllerGroup leftGroup; // All of the left motor controllers grouped together
    private SpeedControllerGroup rightGroup; // All of the right motor controllers grouped together

    private static Drivetrain instance; // This is the instance of the drivetrain class

    private static final double DEADZONE_RANGE = 0.1; // Bounds of the "dead zone" within joystick
    private static final double INCHES_PER_TICK = 2.15812;

    private Drivetrain() {
        l_primary = new CANSparkMax(RobotMap.Drivetrain.LEFT_PRIMARY, MotorType.kBrushless);
        l_secondary = new CANSparkMax(RobotMap.Drivetrain.LEFT_SECONDARY, MotorType.kBrushless);
        l_tertiary = new CANSparkMax(RobotMap.Drivetrain.LEFT_TERTIARY, MotorType.kBrushless);
        r_primary = new CANSparkMax(RobotMap.Drivetrain.RIGHT_PRIMARY, MotorType.kBrushless);
        r_secondary = new CANSparkMax(RobotMap.Drivetrain.RIGHT_SECONDARY, MotorType.kBrushless);
        r_tertiary = new CANSparkMax(RobotMap.Drivetrain.RIGHT_TERTIARY, MotorType.kBrushless);

        leftGroup = new SpeedControllerGroup(l_primary,l_secondary,l_tertiary);
        rightGroup = new SpeedControllerGroup(r_primary,r_secondary,r_tertiary);

        leftGroup.setInverted(false); // TODO: test inversions (copied from 2019-robot, but can never be too safe)
        rightGroup.setInverted(true);

        l_primary.getEncoder()
            .setPositionConversionFactor(INCHES_PER_TICK); // set scale for encoder ticks to inches
        r_primary.getEncoder()
            .setPositionConversionFactor(INCHES_PER_TICK);
    }

    /**
     * Checks to see if the instance of this class has already been created.
     * If so, return it. If not, create it and return it.
     * @return instance of the drivetrain class
     */
    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }

    /**
     * Sets left side of robot to specified speed (between 0 and 1)
     * @param speed speed to set right side of robot to
     */
    public void setLeftSpeed(double speed) {
        leftGroup.set(speed);
    }

    /**
     * Sets right side of robot to specified speed (between 0 and 1)
     * @param speed speed to set right side of robot to
     */
    public void setRightSpeed(double speed) {
        rightGroup.set(speed);
    }
    /**
     * Sets both sides of drivetrain to specified speeds (between 0 and 1)
     * @param leftSpeed speed to set left side of robot to
     * @param rightSpeed speed to set right side of robot to
     */
    public void setSpeed(double leftSpeed, double rightSpeed) {
        setLeftSpeed(leftSpeed);
        setRightSpeed(rightSpeed);
    }

    /**
     * Drives both sides of drivetrain straight for a certain distance
     * @param targetDist the distance to travel
     * @param startPower the desired start speed from -1 to 1
     * @param endPower the desired start speed from -1 to 1
     */
    public void driveStraight(double targetDist, double startPower, double endPower) {
        double pGain = 0.5;
        double initialDist = getAveragePosition();
        double distanceToTarget = Math.abs(targetDist) - Math.abs(getAveragePosition() - initialDist);

        double targetSpeed = pGain*(startPower + ((endPower - startPower) / distanceToTarget));

        if(distanceToTarget > 0) {
            setSpeed(targetSpeed, targetSpeed); // TODO: code sanity check
        }
    }

    /**
     * Arcade drive with movement in y direction controlled by left joystick and movement in x
     * direction controlled by right joystick
     * @param speed The speed to run motors at -- useful for training
     */
    public void arcadeDrive(double speed) {
        double y = OI.driver.getLY();
        double x = OI.driver.getLY();

        y = -1 * deadband(y) * speed;
        x = -1 * deadband(x) * speed;

        setSpeed(y+x, y-x);
    }

    /**
     * Default arcade drive with full speed
     * @author hrl
     */
    public void arcadeDrive() {
        arcadeDrive(1);
    }

    /**
     * Get scaled encoder position. Scale factor is set in constructor.
     * @return left encoder position in inches
     */
    public double getLeftPosition() {
        return l_primary.getEncoder().getPosition();
    }

    /**
     * Get scaled encoder position. Scale factor is set in constructor.
     * @return right encoder position in inches
     */
    public double getRightPosition() {
        return r_primary.getEncoder().getPosition();
    }

    /**
     * Get average scaled encoder position. Scale factor is set in constructor.
     * @return average drivetrain encoder position in inches
     */
    public double getAveragePosition() {
        return (getLeftPosition() + getRightPosition()) / 2;
    }

    public void resetEncoderPositions() {
        l_primary.getEncoder().setPosition(0);
        r_primary.getEncoder().setPosition(0);
    }

    /** 
     * @param value the value of the joystick
     * @return the value of the joystick if it is outside the range of the deadband
     */
    public static double deadband(double value) {
        return Math.abs(value) < DEADZONE_RANGE ? 0 : value;
    }
}
