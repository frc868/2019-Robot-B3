/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import frc.robot.Robot;

/**
 * This class rotates the robot to the specified angle (in degrees) using a Gyro sensor.
 * It uses a PID controller to accomplish this behavior.
 * 
 * @author hrl
 */
public class TurnToAngleGyro {
    private static TurnToAngleGyro instance = null;

    private static final double kP = 0, kI = 0, kD = 0; // TODO: tune this
    private PIDController pid;
    private PIDSource pidSource;
    private PIDOutput pidOutput;

    private TurnToAngleGyro() {
        // initialize sources
        pidSource = new PIDSource() {
            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {
                // shouldn't be used
            }
        
            @Override
            public double pidGet() {
                return Robot.gyro.getAngle();
            }
        
            @Override
            public PIDSourceType getPIDSourceType() {
                return PIDSourceType.kDisplacement;
            }
        };

        pidOutput = new PIDOutput() {        
            @Override
            public void pidWrite(double output) {
                Robot.drivetrain.setSpeed(output, -output); // point turning, assumes proper inversion
            }
        };

        pid = new PIDController(kP, kI, kD, pidSource, pidOutput);
        pid.setAbsoluteTolerance(1.5); // tolerable error in output
    }
   
    /**
     * Checks to see if the instance of this class has already been created.
     * If so, return it. If not, create it and return it.
     */
    public static TurnToAngleGyro getInstance() {
        if (instance == null) {
            instance = new TurnToAngleGyro();
        }
        return instance;
    } 

    /**
     * Turns the robot to the specified angle (in degrees).
     * @param angle the angle to turn to
     */
    public void run(int angle) {
        if (!pid.isEnabled()) {
            this.start();
        }

        double currentAngle = Robot.gyro.getAngle(); // relative, without gyro reset
        pid.setSetpoint(currentAngle + angle);
    }

    /**
     * Starts the PID controller manually.
     * Should never need this.
     */
    public void start() {
        pid.enable();
    }

    /**
     * Stops the PID controller manually.
     */
    public void stop() {
        pid.disable();
    }
}
