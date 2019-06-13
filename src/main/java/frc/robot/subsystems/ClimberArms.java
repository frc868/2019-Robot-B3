/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.RobotMap;

/**
 * This is the class for the former Ramps on the robot. The things on the front that help up climb.
 * It was made a singleton to avoid multiple instantiations of the same sensor.
 */
public class ClimberArms {

    private static ClimberArms instance;
    private Solenoid solenoid;

    private ClimberArms() {
        solenoid = new Solenoid(RobotMap.PCM, RobotMap.ClimberArms.SOLENOID_CHANNEL);
        // always specify PCM ID even if we don't have 2 PCMs, just to be safe.
    }

    /**
     * Checks to see if the instance of this class has already been created.
     * If so, return it. If not, create it and return it.
     * @return instance of the ClimberArms class
     */
    public static ClimberArms getInstance() {
        if(instance == null) {
            instance = new ClimberArms();
        }
        return instance;
    }

    

}
