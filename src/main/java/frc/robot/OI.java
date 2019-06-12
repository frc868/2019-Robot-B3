package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI{
    public static XboxController driver = new XboxController(RobotMap.Controllers.DRIVER_PORT);
    public static XboxController operator = new XboxController(RobotMap.Controllers.OPERATOR_PORT);

    public static void init(){

    } 
}