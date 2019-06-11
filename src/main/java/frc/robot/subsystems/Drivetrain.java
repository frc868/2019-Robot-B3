/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Drivetrain extends Subsystem {
  private CANSparkMax
    l_primary, l_secondary, l_tertiary,
    r_primary, r_secondary, r_tertiary;

  private SpeedControllerGroup leftGroup;
  private SpeedControllerGroup rightGroup;

    public Drivetrain(){
      l_primary = new CANSparkMax(RobotMap.Drivetrain.LEFT_PRIMARY, MotorType.kBrushless);
      l_secondary = new CANSparkMax(RobotMap.Drivetrain.LEFT_SECONDARY, MotorType.kBrushless);
      l_tertiary = new CANSparkMax(RobotMap.Drivetrain.LEFT_TERTIARY, MotorType.kBrushless);
      r_primary = new CANSparkMax(RobotMap.Drivetrain.RIGHT_PRIMARY, MotorType.kBrushless);
      r_secondary = new CANSparkMax(RobotMap.Drivetrain.RIGHT_SECONDARY, MotorType.kBrushless);
      r_tertiary = new CANSparkMax(RobotMap.Drivetrain.RIGHT_TERTIARY, MotorType.kBrushless);

      leftGroup = new SpeedControllerGroup(l_primary,l_secondary,l_tertiary);
      rightGroup = new SpeedControllerGroup(r_primary,r_secondary,r_tertiary);

      // l_primary.setInverted(false); 
      // l_secondary.setInverted(false);
      // l_tertiary.setInverted(false);
      // r_primary.setInverted(true);
      // r_secondary.setInverted(true);
      // r_tertiary.setInverted(true);

      leftGroup.setInverted(false);//TODO: test inversions (copied from 2019-robot, but can never be too safe)
      rightGroup.setInverted(true);
    }

    public void setLeftSpeed(double speed){
      // l_primary.set(speed);
      // l_secondary.set(speed);
      // l_tertiary.set(speed);
      leftGroup.set(speed);
    }

    public void setRightSpeed(double speed){
      // r_primary.set(speed);
      // r_secondary.set(speed);
      // r_tertiary.set(speed);
      rightGroup.set(speed);
    }

    public void setSpeed(double leftSpeed, double rightSpeed){
      setLeftSpeed(leftSpeed);
      setRightSpeed(rightSpeed);
    }

    

  @Override
  public void initDefaultCommand() {}
}
