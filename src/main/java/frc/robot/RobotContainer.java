package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.AutoBlueLimeLeftCmd;
import frc.robot.commands.LimeCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TestCmd;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class RobotContainer{

  //Criação dos controles
  XboxController pilot   = new XboxController(DriverConstants.id_Pilot);
  XboxController copilot = new XboxController(2); 
  //XboxController copilot = new XboxController(2); 

  //Criação dos subsistemas
  Swerve    sb_swerve   = new Swerve();
  Limelight limelight = new Limelight();

  public RobotContainer() {}

  public void configureBindings() {
    
    //Swerve
     
    sb_swerve.setDefaultCommand(new SwerveJoystickCmd(
      sb_swerve,
      () -> (-pilot.getLeftY())  * (pilot.getRightBumper() ? 0.2 : 0.8),
      () ->  pilot.getLeftX()   * (pilot.getRightBumper() ? 0.2 : 0.8),
      () -> (-pilot.getRightX())  * (pilot.getRightBumper() ? 0.2 : 0.8),
      () ->  true,
      () ->  pilot.getBButton()));

  }
  
  public Command getAutonomousCommand() {

    /*return new LimeCmd(
      sb_swerve, 
      () -> (limelight.getX()),
      () -> (limelight.getX3d()),
      () -> (limelight.getY3d()),
    true,
    true);*/

    return new TestCmd(sb_swerve);
    
  }

  

}
