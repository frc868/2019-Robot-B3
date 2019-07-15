/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HatchClaw;



public class Robot extends TimedRobot {

  public static Drivetrain drivetrain = Drivetrain.getInstance();
  public static HatchClaw hatchClaw = HatchClaw.getInstance();

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    OI.update();
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void testInit() {
    OI.init();
  }

  @Override
  public void testPeriodic() {
    Scheduler.getInstance().run();
  }

}
