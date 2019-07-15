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
 * Add your docs here.
 */
public class HatchClaw {

    private static HatchClaw instance;
    private Solenoid claw;

    public static final boolean GRABBED_STATE = false; // TODO: test on/off state solenoid

    private HatchClaw() {
        claw = new Solenoid(RobotMap.Carriage.ACTUATOR);

        claw.set(GRABBED_STATE);
    }

    /**
     * Checks to see if the instance of this class has already been created.
     * If so, return it. If not, create it and return it.
     * @return instance of the ClimberArms class
     */
    public static HatchClaw getInstance() {
        if(instance == null) {
            instance = new HatchClaw();
        }
        return instance;
    }

    /**
     * Grabs the hatch panel (default state)
     */
    public void grab() {
        claw.set(GRABBED_STATE);
    }

    /**
     * Releases the hatch panel (on state)
     */
    public void release() {
        claw.set(!GRABBED_STATE);
    }

    /**
     * Grabs hatch panel if it was released,
     * or releases hatch panel if it was grabbed
     */
    public void toggle() {
        claw.set(!claw.get());
    }

    /**
     * Gets whether solenoid is in the grabbed state. Important to note that
     * solenoid is false when hatch is grabbed. Therefore, returns opposite
     * of solenoid state.
     * @return state of the solenoid
     */
    public boolean isGrabbed() {
        return claw.get() == GRABBED_STATE;
    }
}
