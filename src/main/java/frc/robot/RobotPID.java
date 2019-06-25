/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is an experimental wrapper class for the WPILib PIDController. It provides some helpful
 * functionality for implementing and debugging PID Controllers for any system on the robot.
 * 
 * TODO: This is just an idea, and any feedback is welcome, up to and including "delete this."
 *       A final decision should be made on what to do with this class before integrating it
 *       into production.
 * 
 * @author acr
 */
public class RobotPID {
    private PIDController pid;
    private String name; //A name used to indicate the purpose of this PID loop, e.g. "Tilt"
    private double kP; //the proportional gain
    private double kI; //the integral gain
    private double kD; //the derivative gain
    private PIDSource pidSource; //the data source for the PID loop (should be a sensor of some kind)
    private PIDOutput pidOutput; //the output for the PID loop (usually a motor controller)

    public RobotPID(String name, double kP, double kI, double kD, PIDSource pidSource, PIDOutput pidOutput) {
        this.name = name;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.pidSource = pidSource;
        this.pidOutput = pidOutput;

        this.pid = new PIDController(kP, kI, kD, pidSource, pidOutput);
    }

    /**
     * Gets the PIDController object used by this instance of RobotPID.
     * @return The PIDController object used by this instance of RobotPID.
     */
    public PIDController getPID() {
        return this.pid;
    }

    /**
     * Get the proportional gain used by this PID loop.
     * @return The proportional gain used by this PID loop.
     */
    public double getKP() {
        return this.kP;
    }

    /**
     * Get the integral gain used by this PID loop.
     * @return The integral gain used by this PID loop.
     */
    public double getKI() {
        return this.kI;
    }

    /**
     * Get the derivative gain used by this PID loop.
     * @return The derivative gain used by this PID loop.
     */
    public double getKD() {
        return this.kD;
    }

    /**
     * Get the name of this PID loop.
     * @return The name of this PID loop.
     */
    public String getName() {
        return this.name;
    }

    /**
     * Get the PIDSource object used as the data input for this PID loop.
     * @return The PIDSource object used by this PID loop.
     */
    public PIDSource getPidSource() {
        return this.pidSource;
    }

    /**
     * Get the PIDOutput object used as the otuput for this PID loop.
     * @return The PIDOutput object used by this PID loop.
     */
    public PIDOutput getPidOutput() {
        return this.pidOutput;
    }

    /**
     * Gets the current run status of the PID loop.
     * @return true if the PID loop is currently running, false otherwise
     */
    public boolean getStatus() {
        return this.pid.isEnabled();
    }

    /**
     * Set the proportional gain for this PID loop.
     * @param kP the new desired proportional gain
     */
    public void setKP(double kP) {
        this.kP = kP;
        this.pid.setP(this.kP);
    }

    /**
     * Set the integral gain for this PID loop.
     * @param kI the new desired integral gain
     */
    public void setKI(double kI) {
        this.kI = kI;
        this.pid.setI(this.kI);
    }

    /**
     * Set the derivative gain for this PID loop.
     * @param kD this new desired derivative gain
     */
    public void setKD(double kD) {
        this.kD = kD;
        this.pid.setD(this.kD);
    }

    /**
     * Enables the PID loop.
     */
    public void enable() {
        this.pid.enable();
    }

    /**
     * Disables the PID loop.
     */
    public void disable() {
        this.pid.disable();
    }


    /**
     * When called, this method outputs the following information about this PID loop
     * to the SmartDashboard for tuning purposes:
     * 
     * <ul>
     *  <li>The proportional gain</li>
     *  <li>The integral gain</li>
     *  <li>The derivative gain</li>
     *  <li>An indicator of whether the PID loop is currently running</li>
     * </ul>
     * 
     * The name of this PID loop is used in the key values for each of these indicators,
     * to allow multiple PID loops to be tuned on the same SmartDashboard instance.
     * 
     * This method should only be called in OI.java.
     */
    public void displayTuning() {
        //create key values first, for ease of use in get/set methods
        String kPKey = this.name + " P";
        String kIKey = this.name + " I";
        String kDKey = this.name + " D";
        String runningKey = this.name + " running";

        //get gain values using our getters, so any additional work in the get methods is done
        SmartDashboard.putNumber(kPKey, this.getKP());
        SmartDashboard.putNumber(kIKey, this.getKI());
        SmartDashboard.putNumber(kDKey, this.getKD());
        SmartDashboard.putBoolean(runningKey, this.getStatus());
        /* TODO: Look into using custom Trigger class to create an enable/disable button that is
         *       free from Command Base */

        //get new values from the SmartDashboard and use them to set new PID gains
        double newKP = SmartDashboard.getNumber(kPKey, 0.0);
        double newKI = SmartDashboard.getNumber(kIKey, 0.0);
        double newKD = SmartDashboard.getNumber(kDKey, 0.0);
        this.setKP(newKP);
        this.setKI(newKI);
        this.setKD(newKD);
        
    }

    /**
     * Print the PID loop's current running state, for use on the SmartDashboard/RIOLog.
     */
    public String toString() {
        String toString = "" + this.getStatus();
        return toString;
    }
}
