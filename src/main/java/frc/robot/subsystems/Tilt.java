/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.RobotMap;
import frc.robot.helpers.Helper;
import frc.robot.sensors.PotLimit;

/**
 * Controls the tilt of the carriage, allowing the robot to easily switch between 
 * hatches and balls.
 * 
 * @author hrl
 */
public class Tilt {
    private static Tilt instance = null;

    public static enum TiltPosition {
        LOW, MIDDLE, HIGH;
    }

    private WPI_TalonSRX motor;
    private AnalogPotentiometer potentiometer;
    private Solenoid brake;
    private PotLimit limit;

    private PIDController pid;
    private PIDSource pidSource;
    private PIDOutput pidOutput;

    // BOTH setpoints and potentiometer limits!
    private final double LOWER = 0.85, MIDDLE = 0.831, UPPER = 0.726;
    private final boolean BRAKE_MODE = false;
    private static final double kP = 14, kI = 1, kD = 15; // TODO: tune pid constants

    // this is set true if we're at the upper limit
    private boolean limitPower = false;

    private Tilt() {
        motor = new WPI_TalonSRX(RobotMap.Tilt.MOTOR);
        brake = new Solenoid(RobotMap.Tilt.BRAKE);
        potentiometer = new AnalogPotentiometer(RobotMap.Tilt.POTENTIOMETER);
        limit = new PotLimit(potentiometer, UPPER, LOWER);

        motor.setInverted(true);

        pidSource = new PIDSource() {
            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {
                // should not be used
            }
        
            @Override
            public double pidGet() {
                return getPotPosition();
            }
        
            @Override
            public PIDSourceType getPIDSourceType() {
                return PIDSourceType.kDisplacement;
            }
        };

        pidOutput = new PIDOutput() {
            @Override
            public void pidWrite(double output) {
                setSpeed(Helper.boundValue(-output, -0.3, 0.6));
            }
        };

        pid = new PIDController(kP, kI, kD, pidSource, pidOutput);
        pid.setAbsoluteTolerance(0.001);
    }

    /**
     * Returns a singleton instance of the Tilt class.
     */
    public static Tilt getInstance() {
        if (instance == null) {
            instance = new Tilt();
        }
        return instance;
    }

    /**
     * Sets the tilt motor's speed manually.
     * @param speed the motor speed from -1 to 1
     */
    public void setSpeed(double speed) {
        if (limitPower) {
            motor.set(Helper.boundValue(speed, -0.25, 0.25));
        } else {
            motor.set(Helper.boundValue(speed));
        }
    }

    /**
     * Stops the tilt motor manually.
     */
    public void stop() {
        motor.stopMotor();
    }

    /**
     * Gets the potentiometer's value, for reading motor.
     * @return position of motor according to pot
     */
    public double getPotPosition() {
        return potentiometer.get();
    }

    /**
     * Determines if we've crossed the upper threshold.
     * @return whether the upper limit is crossed
     */
    public boolean getBottomLimit() {
        return limit.getForwardLimit();
    }

    /**
     * Determines if we've crossed the lower threshold.
     * @return whether the lower limit was crossed
     */
    public boolean getTopLimit() {
        return limit.getReverseLimit();
    }

    /**
     * Gets the current status of the tilt brake.
     */
    public boolean getBrake() {
        return brake.get();
    }

    /**
     * Determines if the brake is engaged.
     * @return true if brake is on
     */
    public boolean isBraked() {
        return getBrake() == BRAKE_MODE;
    }

    /**
     * Enables the brake.
     */
    public void startBrake() {
        brake.set(BRAKE_MODE);
    }

    /**
     * Disables the brake.
     */
    public void stopBrake() {
        brake.set(!BRAKE_MODE);
    }

    /**
     * Moves the tilt to a position using PID.
     */
    public void setPosition(TiltPosition pos) {
        if (!pid.isEnabled()) {
            startPid();
        }
        stopBrake();
        this.limitPower = false;
        switch (pos) {
            case LOW:
                pid.setSetpoint(LOWER);
                break;
            case MIDDLE: 
                pid.setSetpoint(MIDDLE);
                break;
            case HIGH:
                this.limitPower = true;
                pid.setSetpoint(UPPER);
                break;
            default:
                break;
        }
    }

    /**
     * Starts the PID controller.
     */
    public void startPid() {
        pid.enable();
    }

    /**
     * Stops the PID controller.
     */
    public void stopPid() {
        pid.disable();
    }
}
