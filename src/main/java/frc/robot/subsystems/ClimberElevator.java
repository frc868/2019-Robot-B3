/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Corresponds to the module driving both the climber lifting us onto HAB 3 and
 * the elevator which lifts the game pieces.
 * 
 * @author HRL
 */
public class ClimberElevator {
    private CANSparkMax primary, secondary; // drive motors
    private Solenoid switcher, elevBrake, climbBrake; // switches between climb/elevator, brakes
    //private IRLimit elevTopLim, elevBotLim, climbTopLim, climbBotLim, climbLimSwitch;
    private Ultrasonic leftHSensor, rightHSensor;

    public ClimberElevator() {
        // initialize motor controllers
        primary = new CANSparkMax(RobotMap.ClimberElevator.PRIMARY, MotorType.kBrushless);
        secondary = new CANSparkMax(RobotMap.ClimberElevator.SECONDARY, MotorType.kBrushless);
       
        primary.setIdleMode(IdleMode.kBrake);
        secondary.setIdleMode(IdleMode.kBrake);

        secondary.follow(primary);

        primary.setInverted(RobotMap.ClimberElevator.PRIMARY_IS_INVERTED);
        secondary.setInverted(RobotMap.ClimberElevator.SECONDARY_IS_INVERTED);
        
        // initialize solenoids
        switcher = new Solenoid(RobotMap.PCM, RobotMap.ClimberElevator.SWITCHER);
        elevBrake = new Solenoid(RobotMap.PCM, RobotMap.ClimberElevator.ELEV_BRAKE);
        climbBrake = new Solenoid(RobotMap.PCM, RobotMap.ClimberElevator.CLIMB_BRAKE);

        // initialize limits
        // TODO: write sensors for these
        //elevTopLim = new IRLimit(RobotMap.ClimberElevator.ELEV_TOP_LIM);
        //elevBotLim = new IRLimit(RobotMap.ClimberElevator.ELEV_BOT_LIM);
    }

    /**
     * sets speed of the powerpack motors assuming limits are not tripped
     * @param speed % power from -1 to +1
     */
    public void setSpeed(double speed) {
        speed = Math.min(Math.max(speed, -1), 1);

        primary.set(speed);
    }

    /**
     * stops the motor (and hence the assembly) regardless of state
     */
    public void stop() {
        primary.stopMotor();
    }
}
