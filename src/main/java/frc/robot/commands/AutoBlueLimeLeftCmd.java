package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.function.Supplier;
import java.util.Vector;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.subsystems.LimelightHelpers.PoseEstimate;

public class AutoBlueLimeLeftCmd extends Command{
    
    Swerve sb_swerve;
    Limelight limelight = new Limelight();
    double xSpeed, ySpeed, turningSpeed, gyro;
    int steps = 0;
    Supplier<Double> turningSpdFunction, xSpdFunction, ySpdFunction;
    PIDController xSpd, ySpd, turningSpd;
    //Quadra
    Boolean fieldOrientedFunction, resetGyro, doRejectUpdate = false;
    //Suaviza transições rápidas
    SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    Boolean resetBlock = false, setPoint1, setPoint2, isFinished = false;
  
    public AutoBlueLimeLeftCmd(Swerve sb_swerve,  Supplier<Double> turningSpdFunction, Supplier<Double> xSpdFunction, 
    Supplier<Double> ySpdFunction, Boolean fieldOrientedFunction, Boolean resetGyro, Boolean setPoint1, Boolean setPoint2) {
  
      this.setPoint1 = setPoint1;
      this.setPoint2 = setPoint2;

         xSpd    = new PIDController(.25, 0.000000000000001, 0.0003);
         ySpd    = new PIDController(.00006, 0, 0.0003);
      turningSpd = new PIDController(.008, 0, 0);
         gyro    = sb_swerve.getHeading();
  
      this.sb_swerve             = sb_swerve;
      this.xSpdFunction          = xSpdFunction;
      this.ySpdFunction          = ySpdFunction;
      this.turningSpdFunction    = turningSpdFunction;
      this.fieldOrientedFunction = fieldOrientedFunction; 
      this.resetGyro             = resetGyro;
  
      addRequirements(sb_swerve);
      
      LimelightHelpers.SetRobotOrientation("limelight", 0, 0, 0, 0, 0, 0);
      
    }
  
    @Override
    public void initialize() {}
  
    @Override
    public void execute() {

      SmartDashboard.putNumber("limeOrientation", LimelightHelpers.getBotPose3d("limelight").getX());
      SmartDashboard.putNumber("limeOrientationn", limelight.getX());
      SmartDashboard.putNumber("lime X", limelight.getX3d());
      SmartDashboard.putNumber("lime Y", limelight.getY3d());
      SmartDashboard.putNumber("LimeXCalculate", xSpd.calculate(xSpeed));
      SmartDashboard.putNumber("LimeYCalculate", ySpd.calculate(ySpeed));
      SmartDashboard.putNumber("LimeTurningCalculate", turningSpd.calculate(turningSpeed));
      SmartDashboard.putNumber("steps", steps);
      SmartDashboard.putBoolean("targets", limelight.noTag());
  
      //Obtém valores analógicos
      turningSpeed = turningSpdFunction.get();
      xSpeed       = xSpdFunction.get();
      ySpeed       = ySpdFunction.get();
      
      //Definindo Setpoints
      if(setPoint1){
        xSpd.setSetpoint(setSetPointX1());
        ySpd.setSetpoint(setSetPointY1());
      } else if(setPoint2){
        xSpd.setSetpoint(setSetPointX2());
        ySpd.setSetpoint(setSetPointY2());
      } 
      turningSpd.setSetpoint(0);
      //turnGyro.setSetpoint(.9);

      verifySetpoint();

      SmartDashboard.putNumber("errorX", xSpd.getError());
      SmartDashboard.putNumber("errorY", ySpd.getError());
      SmartDashboard.putBoolean("isOn", isFinished);
      SmartDashboard.putBoolean("setpoint2", setPoint2);

      //Criação da velocidade do chassi desejada
      ChassisSpeeds chassisSpeeds;
  
      //Verifica se a direçaõ é em relação a quadra
      if(!limelight.noTag() && setPoint1){
        chassisSpeeds = new ChassisSpeeds(ySpd.calculate(ySpeed), xSpd.calculate(xSpeed), turningSpd.calculate(turningSpeed));
        //chassisSpeeds = new ChassisSpeeds(0, 0, 0);
        limelight.setLedOn(true);
  
      } else if(!limelight.noTag() && setPoint2){
        chassisSpeeds = new ChassisSpeeds(-ySpd.calculate(ySpeed), -xSpd.calculate(xSpeed), turningSpd.calculate(turningSpeed));
        //chassisSpeeds = new ChassisSpeeds(0, 0, 0);
        limelight.setLedOn(true);
      }
        else {
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, sb_swerve.getRotation2d());
        limelight.setLedOn(false);
      }
      
      //Converte a velocidade do chassi ao estado individual dos módulos
      SwerveModuleState[] moduleStates = SwerveConstants.kinematics.toSwerveModuleStates(chassisSpeeds);
  
      //Seta cada estado para suas rodas
      sb_swerve.setModuleStates(moduleStates);
    }
  
    @Override
    public void end(boolean interrupted) {
      //Para Módulos
      sb_swerve.stopModules();
    }
  
    @Override
    public boolean isFinished() {
      return isFinished;
    }
  
    public double setSetPointX1(){
      double xSetpoints1[] = {3.66, 3.65};
      return xSetpoints1[steps];
    }
    public double setSetPointY1(){
      double ySetpoints1[] = {-1.15, -1.14};
      return ySetpoints1[steps];
    }
    public double setSetPointX2(){
      double xSetpoints2[] = { 6.33,   4.77,  6.33,  4.77};
      return xSetpoints2[steps];
    }
    public double setSetPointY2(){
      double ySetpoints2[] = {-2.71,  -0.95, -2.71, -0.95};
      return ySetpoints2[steps];
    }

    public void verifySetpoint(){
      if (setPoint1 && (Math.abs(xSpd.getError()) < .3 && Math.abs(ySpd.getError()) < .1)) {
        if(steps == 1) {
          isFinished = true;
        } else { 
          steps++;
        }
      }
      if (setPoint2 && (Math.abs(xSpd.getError()) < .4 && Math.abs(ySpd.getError()) < .1)) {
        if(steps == 3) {
          isFinished = true;
        } else { 
          steps++;
        }
      }
    }
}