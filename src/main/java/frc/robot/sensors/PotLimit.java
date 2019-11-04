/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

/**
 * Utilizes a potentiometer as a limit switch. Useful for tilt.
 * We do not live in Colorado.
 * 
 * @author hrl
 */
public class PotLimit {
    private AnalogPotentiometer potentiometer;
    private final double FORWARD_LIMIT, REVERSE_LIMIT;

    public PotLimit(AnalogPotentiometer pot, double fwdLim, double revLim) {
        this.potentiometer = pot;
        this.FORWARD_LIMIT = fwdLim;
        this.REVERSE_LIMIT = revLim;
    }

    public boolean getForwardLimit() {
        return potentiometer.get() > FORWARD_LIMIT;
    }

    public boolean getReverseLimit() {
        return potentiometer.get() < REVERSE_LIMIT;
    }
}
