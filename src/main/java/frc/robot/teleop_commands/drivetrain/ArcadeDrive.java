/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.teleop_commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.helpers.Helper;

public class ArcadeDrive extends Command {
  public ArcadeDrive() {
    requires(Robot.drivetrain);
  }

  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //TODO: add conditions for slow mode
    double y = OI.driver.getRawAxis(RobotMap.Controllers.LY); //TODO: Check this
    double x = OI.driver.getRawAxis(RobotMap.Controllers.RX);

    y = Helper.deadband(y, Helper.DEADZONE_RANGE);
    x = Helper.deadband(x, Helper.DEADZONE_RANGE);

    Robot.drivetrain.setSpeed(y+x, y-x);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
