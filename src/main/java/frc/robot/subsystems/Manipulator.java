/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.sensors.IRLimit;

/**
 * The manipulator class. Governs both the hatch and ball intakes.
 * 
 * @author dri
 */
public class Manipulator {
    private static Manipulator instance;
    private Solenoid claw;
    private WPI_TalonSRX ballIntake;
    private IRLimit ballDetect;

    public static final boolean GRABBED_STATE = false; // TODO: test on/off state solenoid
    public static final boolean BALL_DETECTED_STATE = true; // state the IR limit is in when the ball is in

    private Manipulator() {
        claw = new Solenoid(RobotMap.Manipulator.ACTUATOR);
        ballIntake = new WPI_TalonSRX(RobotMap.Manipulator.INTAKE_MOTOR);
        ballDetect = new IRLimit(RobotMap.Manipulator.BALL_IR_LIMIT);

        claw.set(GRABBED_STATE);
    }

    /**
     * Checks to see if the instance of this class has already been created.
     * If so, return it. If not, create it and return it.
     * @return instance of the ClimberArms class
     */
    public static Manipulator getInstance() {
        if(instance == null) {
            instance = new Manipulator();
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

    /**
     * Run the ball intake based on the values of driver and operator triggers.
     * Polls the controllers in OI, sets speed accordingly.
     * Right trigger is intake, left trigger is outtake.
     */
    public void manualIntake() {
        double rightOperatorTrigger = OI.operator.getRT();
        double leftOperatorTrigger = OI.operator.getLT();

        double rightDriverTrigger = OI.driver.getRT();
        double leftDriverTrigger = OI.driver.getLT();

        ballIntake.set(
            leftOperatorTrigger
            - rightOperatorTrigger
            + leftDriverTrigger
            - rightDriverTrigger
        );
    }

    /**
     * Checks if the ball has been intaked using the limit switch.
     * @author hrl
     */
    public boolean isBallDetected() {
        return ballDetect.get() == BALL_DETECTED_STATE;
    }
}
