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
 * @author dri
 */
public class ClimberArms {

    private static ClimberArms instance;
    private Solenoid arms;
    private final boolean DEPLOYED_STATE = true; //TODO: test deployed state for solenoid

    private ClimberArms() {
        arms = new Solenoid(RobotMap.PCM, RobotMap.ClimberArms.SOLENOID_CHANNEL);
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
    /**
     * sets the state of the solenoid to deployed or secured
     * true = deployed
     * false = secured
     * @param state the state to set it to
     */
    public void setState(boolean state) {
        arms.set(state);
    }

    /**
     * sets the state of the solenoid to true, which is the "deployed" state
     */
    public void deploy() {
        arms.set(DEPLOYED_STATE);
    }

    /**
     * sets the state of the solenoid to false, which is the "secured" state
     */
    public void secure() {
        arms.set(!DEPLOYED_STATE);
    }
    
    /**
     * returns the state of the climber arms solenoid
     * if true, deployed
     * if false, secured
     * @return state of solenoid
     */
    public boolean getState() {
        return arms.get();
    }

}
