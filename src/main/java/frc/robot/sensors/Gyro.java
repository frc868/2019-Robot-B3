/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogGyro;
import frc.robot.RobotMap;

/**
 * This class represents the main Gyro for the robot. It was made a singleton to
 * avoid multiple instantiations of the same sensor.
 * 
 * @author acr
 */
public class Gyro {

    private static Gyro instance = null;    //This is the instance of our Gyro class.
    private AnalogGyro gyro = null;  //This is the actual Gyro class from WPI which interface with the sensor.

    /**
     * This is the constructor for our Gyro class. It is private because
     * this is a singleton, and it should never be instantiated outside of
     * the getInstance() method.
     */
    private Gyro() {
        gyro = new AnalogGyro(RobotMap.Sensors.GYRO);        
    }

    /**
     * Checks to see if the instance of this class has already been created.
     * If so, return it. If not, create it and return it.
     */
    public static Gyro getInstance() {
        if(instance == null) {
            instance = new Gyro();
        }

        return instance;
    }

    /**
     * Get the current gyro angle, in degrees.
     */
    public double getAngle() {
        /* Note: If we need to manipulate the data from the gyro before returning it for use,
         *       we can do it here. */

        return gyro.getAngle();
    }

    /**
     * Reset the gyro to zero degrees.
     */
    public void reset() {
        gyro.reset();
    }

    /**
     * Print the gyro's current angle, for use on the SmartDashboard.
     */
    @Override
    public String toString() {
        String toString = "" + this.getAngle();
        return toString;
    }
}
