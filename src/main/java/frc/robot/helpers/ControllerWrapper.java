/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.helpers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import frc.robot.RobotMap;

/**
 * A wrapper for WPILib's XboxController class. Importantly, it adds the ability
 * to retrieve a WPILib Button.
 *  
 * Utilized by instiating a new ControllerWrapper rather than an XboxController.
 * Any XboxController-specific submethods can be retrieved by
 * <code>driver.getController().getRawAxis(1)</code> or the like. 
 * Buttons/D-pad directions are referenced using <code>driver.bA</code> or
 * <code>driver.dNE</code>, for example.
 *  
 * @author hrl
 */
public class ControllerWrapper {
    private XboxController controller;
    public JoystickButton bA, bB, bX, bY, bRB, bLB, bRSTK, bLSTK, bSTART, bMENU; // read: "button Menu"
    public POVButton dN, dE, dS, dW, dNE, dNW, dSE, dSW; // read: "d-pad North"

    private Timer timer; // used for rumble timing
    private final double RUMBLE_DELAY = 0.3;

    public ControllerWrapper(int port) {
        controller = new XboxController(port);

        // face buttons
        bA = new JoystickButton(this.controller, RobotMap.Controllers.A);
        bB = new JoystickButton(this.controller, RobotMap.Controllers.B);
        bX = new JoystickButton(this.controller, RobotMap.Controllers.X);
        bY = new JoystickButton(this.controller, RobotMap.Controllers.Y);
        bSTART = new JoystickButton(this.controller, RobotMap.Controllers.START);
        bMENU = new JoystickButton(this.controller, RobotMap.Controllers.MENU);

        // misc buttons
        bRB = new JoystickButton(this.controller, RobotMap.Controllers.RB);
        bLB = new JoystickButton(this.controller, RobotMap.Controllers.LB);
        bRSTK = new JoystickButton(this.controller, RobotMap.Controllers.RSTK);
        bLSTK = new JoystickButton(this.controller, RobotMap.Controllers.LSTK);

        // d-pad/"POV" buttons
        // reads angle value of the combined d-pad
        dN = new POVButton(this.controller, 0);
        dNE = new POVButton(this.controller, 45);
        dE = new POVButton(this.controller, 90);
        dSE = new POVButton(this.controller, 135);
        dS = new POVButton(this.controller, 180);
        dSW = new POVButton(this.controller, 225);
        dW = new POVButton(this.controller, 270);
        dNW = new POVButton(this.controller, 315);
    }

    /**
     * Grants access to the base XboxController, for additional methods.
     * @return an XboxController object
     */
    public XboxController getController() {
        return this.controller;
    }

    /**
     * Returns the left stick's X-axis value.
     */
    public double getLX() {
        return this.controller.getRawAxis(RobotMap.Controllers.LX);
    }

    /**
     * Returns the left stick's Y-axis value.
     */
    public double getLY() {
        return this.controller.getRawAxis(RobotMap.Controllers.LY);
    }

    /**
     * Returns the right stick's X-axis value.
     */
    public double getRX() {
        return this.controller.getRawAxis(RobotMap.Controllers.RX);
    }

    /**
     * Returns the right stick's Y-axis value.
     */
    public double getRY() {
        return this.controller.getRawAxis(RobotMap.Controllers.RY);
    }

    /**
     * Returns the left trigger's value.
     */
    public double getLT() {
        return this.controller.getRawAxis(RobotMap.Controllers.LT);
    }

    /**
     * Returns the right trigger's value.
     */
    public double getRT() {
        return this.controller.getRawAxis(RobotMap.Controllers.RT);
    }

    /**
     * Rumbles the controller for a given amount of time.
     * NEEDS TESTING.
     */
    public void rumble() {
        timer.reset();
        timer.start();

        this.controller.setRumble(RumbleType.kLeftRumble, 1);
        this.controller.setRumble(RumbleType.kRightRumble, 1);
        while (timer.get() < RUMBLE_DELAY) {} // idle
        this.controller.setRumble(RumbleType.kLeftRumble, 0);
        this.controller.setRumble(RumbleType.kRightRumble, 0);

        timer.stop();
    }
}
