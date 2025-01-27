package frc.robot.commands;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LimelightHelpers.PoseEstimate;

public class LimeCmd extends Command {

  //TODO Criação dos comandos do controle da swerve

  //Subsistema
  Swerve sb_swerve;
  Limelight limelight = new Limelight();
  double xSpeed, ySpeed, gyro;
  Supplier<Double> turningSpdFunction, xSpdFunction, ySpdFunction;
  PIDController xSpd, ySpd, turningSpd;
  //Quadra
  Boolean fieldOrientedFunction, resetGyro, doRejectUpdate = false;
  //Suaviza transições rápidas
  SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  Boolean resetBlock = false;

  public LimeCmd(Swerve sb_swerve,  Supplier<Double> turningSpdFunction, Supplier<Double> xSpdFunction, 
  Supplier<Double> ySpdFunction, Boolean fieldOrientedFunction, Boolean resetGyro) {

       xSpd    = new PIDController(.35, 0.000000000000005, 0.1);
       ySpd    = new PIDController(.35, 0.000000000000005, 0.1);
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
    LimelightHelpers.setPriorityTagID("limelight", 11);
    
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    //int Distance = int(LimelightHelpers.getBotPose3d_TargetSpace("limelight").getMeasureX());
    //SmartDashboard.putNumber("limelight getBotPose", LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight"));
    /*SmartDashboard.putNumber("limeOrientation", LimelightHelpers.getBotPose3d("limelight").getX());
    */SmartDashboard.putNumber("limeOrientation", limelight.getTrackX());
    SmartDashboard.putNumber("lime X", limelight.getX3d());
    SmartDashboard.putNumber("lime Y", limelight.getY3d());
    SmartDashboard.putNumber("LimeXCalculate", xSpd.calculate(xSpeed));
    SmartDashboard.putNumber("LimeYCalculate", ySpd.calculate(ySpeed));
    /*SmartDashboard.putBoolean("targets", limelight.noTag());
    *///SmartDashboard.putNumber("LimeTRNGCalculate", turningSpd.calculate(turningSpeed));

    //Obtém valores analógicos
    double turningSpeed = turningSpdFunction.get();
    xSpeed = xSpdFunction.get();
    ySpeed = ySpdFunction.get();
    
    //Definindo Setpoints
    xSpd.setSetpoint(3.4);
    ySpd.setSetpoint(-1.13);
    //turningSpd.setSetpoint(0);
    //xSpd.setSetpoint(xSetpointReef1);
    //ySpd.setSetpoint(ySetpointFeeder);
    turningSpd.setSetpoint(0);

    //Criação da velocidade do chassi desejada
    ChassisSpeeds chassisSpeeds;

    //Verifica se a direçaõ é em relação a quadra
    if(!limelight.noTag()){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpd.calculate(ySpeed), xSpd.calculate(xSpeed), turningSpd.calculate(turningSpeed), sb_swerve.getRotation2d());
      ///chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, sb_swerve.getRotation2d());
      limelight.setLedOn(true);

    } else {
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
    return false;
  }
}