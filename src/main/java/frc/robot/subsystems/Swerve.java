package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

//TODO Definição dos módulos swerve e criação da odometria:

public class Swerve extends SubsystemBase {

  //Definição sos módulos
  SwerveModule backLeft   = new SwerveModule(1, 2, false, true, 9, 0);
  SwerveModule backRight  = new SwerveModule(4, 3, false, true, 10, 0);
  SwerveModule frontLeft  = new SwerveModule(8, 7, false, true, 11, 0);
  SwerveModule frontRight = new SwerveModule(5, 6, false, true, 12, 0);

  //Obtém posições dos módulos
  SwerveModulePosition[] modulePositions = {frontLeft.getSwerveModulePosition(), frontRight.getSwerveModulePosition(), 
                                            backLeft.getSwerveModulePosition(), backRight.getSwerveModulePosition()};

  //Criação do Giroscópio                                          
  Pigeon2 pigeon = new Pigeon2(13);

  //Criaçaõ da Odometria
  SwerveDriveOdometry odometer = new SwerveDriveOdometry(SwerveConstants.kinematics, getRotation2d(), modulePositions);

  //Offset da posição inicial do robô
  double gyroOffset = 0;

  //Pausa para estabilização do sistema e resetar o giroscópio adequadamente
  public Swerve() {
    new Thread(() -> {
      try {
          Thread.sleep(1000);
          zeroHeading();
      } catch (Exception e) {
      }
  }).start();
  }

  //Função de offset da posiçaõ inicial
  public void setGyroOffset (double offset) {
    gyroOffset = offset;
  }

  //Função de resetagem do Giroscópio
  public void zeroHeading(){
    pigeon.reset();
  }

  //Obtém a posição do giroscópio em relação a quadra
  public double getHeading(){
    return Math.IEEEremainder((pigeon.getRotation2d().getDegrees() + gyroOffset) % 360, 360.0);
  }

  //Obtém a angulação da quadra 
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(-getHeading());
  }
  
  //Obtém a posição do robô em um plano 2D 
  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  //FunçãO de resetagem da odometria
  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), modulePositions, pose);
  }

  //Funçaõ para parar módulos
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  //Funçaõ de definição de cada estado de módulo
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 5);
    frontRight.setDesiredState(desiredStates[0]);
    frontLeft.setDesiredState(desiredStates[1]);
    backRight.setDesiredState(desiredStates[2]);
    backLeft.setDesiredState(desiredStates[3]);
  }

  @Override
  public void periodic() {
    //Atualiza odometria
    odometer.update(getRotation2d(), modulePositions);

    //SmartDashboard.putNumber("frontLeft", frontLeft.getTurningPosition()/360.);
    //SmartDashboard.putNumber("frontRight", frontRight.getTurningPosition()/360.);
    //SmartDashboard.putNumber("backLeft", backLeft.getTurningPosition()/360.);
    //SmartDashboard.putNumber("backRight", backRight.getTurningPosition()/360.);

    SmartDashboard.putNumber("HeadingPigeon", -Math.IEEEremainder((pigeon.getRotation2d().getDegrees() + gyroOffset) % 360, 360.0));
    SmartDashboard.putNumber("pigeon", -pigeon.getRotation2d().getDegrees());

  }
}
