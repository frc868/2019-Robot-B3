/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import frc.robot.RobotMap;
import frc.robot.sensors.IRLimit;

/**
 * Corresponds to the module driving both the climber lifting us onto HAB 3 and
 * the elevator which lifts the game pieces.
 * 
 * @author hrl
 */
public class ClimberElevator {
    private static ClimberElevator instance;

    private CANSparkMax primary, secondary; // drive motors
    private WPI_TalonSRX footMotor; // the bottom motor on the foot to drive forward
    private Solenoid switcher, elevBrake, climbBrake; // switches between climb/elevator, brakes
    private IRLimit elevTopLim, elevBotLim, climbTopLim, climbBotLim, climbLimSwitch;
    private Ultrasonic leftHSensor, rightHSensor;
    
    private final boolean ELEVATOR_MODE = false; // switcher mode to activate elevator
    private final boolean BRAKE_MODE = false;

    private static final double kP = 0.00, kI = 0.00, kD = 0.00; // TODO: tune PID constants
    private PIDController controller; // the pid controller for the elevator
    private PIDSource source; // source for pid controller
    private PIDOutput output; // output for pid controller

    private static final double
        INTAKE_BALL = 2.54, LOWER_BALL = 5.02, MIDDLE_BALL = 22.023, UPPER_BALL = 39,
        LOWER_HATCH = 0.5, MIDDLE_HATCH = 19.66, UPPER_HATCH = 35.85;
        // target encoder counts of the primary elevator motor at each height necessary for game

    private ClimberElevator() {
        // initialize motor controllers
        primary = new CANSparkMax(RobotMap.ClimberElevator.PRIMARY, MotorType.kBrushless);
        secondary = new CANSparkMax(RobotMap.ClimberElevator.SECONDARY, MotorType.kBrushless);

        footMotor = new WPI_TalonSRX(RobotMap.ClimberElevator.FOOT_MOTOR);
       
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
        elevTopLim = new IRLimit(RobotMap.ClimberElevator.ELEV_TOP_LIM);
        elevBotLim = new IRLimit(RobotMap.ClimberElevator.ELEV_BOT_LIM);
        climbLimSwitch = new IRLimit(RobotMap.ClimberElevator.CLIMB_LIM_SWITCH);

        source = new PIDSource(){ // sensor source for the elevator pid controller
        
            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {
                
            }
        
            @Override
            public double pidGet() {
                return primary.getEncoder().getPosition(); // not using scaling, just raw counts
            }
        
            @Override
            public PIDSourceType getPIDSourceType() {
                return PIDSourceType.kDisplacement; // based on encoder position, not velocity
            }
        };
        output = new PIDOutput(){ // setting motor output to output of pid loop
        
            @Override
            public void pidWrite(double output) {
                primary.set(output);
            }
        };
        controller = new PIDController(kP, kI, kD, source, output);
    }

    /**
     * Checks to see if the instance of this class has already been created.
     * If so, return it. If not, create it and return it.
     * @return instance of the ClimberElevator class
     */
    public static ClimberElevator getInstance() {
        if (instance == null) {
            instance = new ClimberElevator();
        }
        return instance;
    }

    public void periodic() {
        if (primary.get() == 0) {
            setElevBrake(true);
        }
    }

    /**
     * sets speed of the powerpack motors assuming limits are not tripped
     * @param speed % power from -1 to +1
     * @author dri
     */
    public void setSpeed(double speed) {
        if (elevTopLim.get()) {
        }
        else if (elevBotLim.get()) {
        }
        else {
        }

        setElevBrake(speed == 0.0);

        primary.set(-speed);
    }

    /**
     * stops the motor (and hence the assembly) regardless of state
     */
    public void stop() {
        primary.stopMotor();
    }

    /**
     * gets the raw encoder position of the elevator (one tick per motor revolution)
     */
    public double getElevatorPosition() {
        return primary.getEncoder().getPosition();
    }

    /**
     * gets raw encoder speed of elevator
     * @return speed of elevator
     */
    public double getElevatorSpeed() {
        return primary.getEncoder().getVelocity();
    }

    /**
     * switches the mechanism to elevator mode
     */
    public void switchToElevator() {
        switcher.set(ELEVATOR_MODE);
    }

    /**
     * switches the mechanism to climber mode
     */
    public void switchToClimber() {
        switcher.set(!ELEVATOR_MODE);
    }

    /** 
     * checks whether the mechanism is in elevator mode or climber mode
     * does NOT return the raw value of the switcher
     * @return true if in elevator, false if in climber
     */
    public boolean getSwitcher() {
        return switcher.get() == ELEVATOR_MODE;
    }

    /**
     * sets the elevator brake to on or off
     * @param state whether brake should be on (true) or off (false)
     */
    public void setElevBrake(boolean state) {
        if (state) {
            elevBrake.set(BRAKE_MODE);
        } else {
            elevBrake.set(!BRAKE_MODE);
        }
    }

    /**
     * gets the state of the elevator brake
     * @return true if brake on, false if brake off
     */
    public boolean getElevBrake() {
        return elevBrake.get() == BRAKE_MODE;
    }

    /**
     * sets the climber brake to on or off
     * @param state whether brake should be on (true) or off (false)
     */
    public void setClimbBrake(boolean state) {
        if (state) {
            elevBrake.set(BRAKE_MODE);
        } else {
            elevBrake.set(!BRAKE_MODE);
        }
    }

    /**
     * gets the state of the climber brake
     * @return true if brake on, false if brake off
     */
    public boolean getClimbBrake() {
        return climbBrake.get() == BRAKE_MODE;
    }

    public boolean getTopLimit() {
        return elevTopLim.get();
    }

    public boolean getBotLimit() {
        return elevBotLim.get();
    }

    /**
     * sets the foot motor to a value between -1 and 1
     * @param speed the power to set the motor to
     */
    public void driveFoot(double speed) {
        footMotor.set(speed);
    }
   
    /**
     * prints the current state of the climber-elevator assembly for use
     * on the SmartDashboard
     */
    @Override
    public String toString() {
        return "I don't know yet?";
    }
}
