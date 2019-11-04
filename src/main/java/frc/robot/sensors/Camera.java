/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.helpers.Helper;

/**
 * Controls the Limelight camera connected to the robot over NetworkTables.
 * 
 * @author hrl
 */
public class Camera {
    private static Camera instance = null; // the instance to be used for getInstance()

    private NetworkTable table;
    // TODO: figure out what these are (@skyler)
    private NetworkTableEntry tv, ta, tx, ts;

    // TODO: tune these! copied from 2019-Robot
    private static final double kDist = 0.18;
    private static final double kPos = 0.008;
    private static final double kArea = 0.1;

    public Camera() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tv = table.getEntry("tv");
        ta = table.getEntry("ta");
        tx = table.getEntry("tx");
        ts = table.getEntry("ts");
    }

    /**
     * Gets the singleton instance of the Camera class.
     */
    public static Camera getInstance() {
        if (instance == null) {
            instance = new Camera();
        }
        return instance;
    }

    /**
     * Determines whether the target has been found.
     * @return boolean representing target detection status
     */
    public boolean hasTarget() {
        return tv.getDouble(0.0) == 1;
    }

    /**
     * Returns the area of the target, if detected.
     * @return 0.0 if no target, area otherwise
     */
    public double getArea() {
        return ta.getDouble(0.0);
    }

    /**
     * Gets the x-position of the target, if detected.
     * @return 0.0 if no target, x-position otherwise
     */
    public double getPosition() {
        return tx.getDouble(0.0);
    }

    /**
     * Gets the angle (with respect to the horizontal axis) of the target, if detected.
     * @return 0.0 if no target, x-angle otherwise
     */
    public double getAngle() {
        double angle = ts.getDouble(0.0);
        if (angle < -45) {
            angle += 90;
        }
        return angle;
    }

    /**
     * Updates the robot's angle and position actively to correct for alignment error.
     * Useful for auton.
     */
    public void followVision() {
        if (this.hasTarget()) {
            double area = this.getArea();
            double posError = this.getPosition(); // how far we are from the target
            // the target value we are going to
            double posValue = posError * kPos * Math.sqrt(Helper.boundValue((area * kArea), 0, 1));

            // powers to set drivetrain to
            double left = Helper.boundValue((1/Math.sqrt(area)) * kDist + posValue);
            double right = Helper.boundValue((1/Math.sqrt(area)) * kDist + posValue);

            Robot.drivetrain.setSpeed(left, right);
        }
    }

    /**
     * Update only the robot's angle actively to correct for alignment error.
     * Useful for teleop.
     */
    public void manualFollowVision() {
        if (this.hasTarget()) {
            double area = this.getArea();
            double posError = this.getPosition(); // how far we are from the target
            // the target value we are going to
            double posValue = posError * kPos * Math.sqrt(Helper.boundValue((area * kArea), 0, 1));

            double angleError = this.getAngle(); // how disoriented we are from the target
            double angleValue = 0.0; // the target value we are going to

            // y movement on driver controller
            double y = Helper.deadzone(-OI.driver.getLY(), 0.03);

            // powers to set drivetrain to
            double left = y + (posValue + angleValue);
            double right = y - (posValue + angleValue);

            Robot.drivetrain.setSpeed(left, right);
        }
    }

    /**
     * Prints the position, angle, and target status, for use on the SmartDashboard.
     * @return "Position,Angle,TargetFound"
     */
    @Override
    public String toString() {
        String toString = "" + this.getPosition() + "," + this.getAngle() + "," + this.hasTarget();
        return toString;
    }
}
