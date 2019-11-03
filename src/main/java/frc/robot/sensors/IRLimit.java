/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * This class represents an instance of the infrared limit switches used on our
 * robot. These are digital inputs that change state when an object comes within
 * close proximity (around 1 inch) to the limit switch.
 * 
 * @author acr
 */
public class IRLimit {

    private DigitalInput limitSwitch;

    /**
     * Constructs a new IR limit switch object.
     * 
     * @param channel The channel number that the IR limit switch is wired into on the RoboRIO.
     */
    public IRLimit(int channel) {
        this.limitSwitch = new DigitalInput(channel);
    }

    /**
     * Get the current state of the limit switch.
     */
    public boolean get() {
        /* Note: If we need to manipulate the data before returning it for use (i.e. invert it),
         *       we can do it here.*/
        
        return !this.limitSwitch.get();
    }

    /**
     * Print the state of the limit switch, for use on the SmartDashboard.
     */
    @Override
    public String toString() {
        String toString = "" + this.get();
        return toString;
    }
}
