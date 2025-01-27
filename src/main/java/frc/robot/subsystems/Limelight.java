package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  //Criação da Limelight
  NetworkTable      table = NetworkTableInstance.getDefault().getTable("limelight");
  //table = NetworkTableInstance.getDefault().set
  NetworkTableEntry tx    = table.getEntry("tx");
  NetworkTableEntry ty    = table.getEntry("ty");
  NetworkTableEntry ta    = table.getEntry("ta");
  NetworkTableEntry tid   = table.getEntry("tid");

  double x, y, area, id, xTele;

  public Limelight() {
    
  }

  //Função de setagem das luzes da Limelight
  public void setMode (int mode) {
    table.getEntry("ledMode").setNumber(mode);
  }
  

  //Obtém eixo X em relação a AprilTag
  public double getX () {
    return x;
  }

  //Obtém proprocional para o ajuste do robô em relação ao eixo X da AprilTag
  public double getTrackX() {
    return x * 0.015;
  }
  public double getTrackTeleX() {
    return xTele * 0.013;
  }

  //Obtém eixo Y em relação a AprilTag
  public double getY () {
    return y;
  }

  //Obtém proprocional para o ajuste do robô em relação ao eixo Y da AprilTag
  public double getTrackY() {
    return y;
  }

  //Obtém id da AprilTag
  public Boolean tagSpeaker () {
    return id == 6 || id == 11;
  }

  //Obtém AprilTag identificada
  public Boolean noTag () {
    return id <= 0;
  }

  @Override
  public void periodic() {
 
    //Localização e id da AprilTag
    x    = tx.getDouble(0.0);
    xTele = tx.getDouble(0.0);
    y    = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    id   = tid.getDouble(0.0);
    
    //post to smart dashboard periodically
    //SmartDashboard.putNumber("getTrackx", getTrackX()); 
    //SmartDashboard.putNumber("getTracky", getTrackY());
    //SmartDashboard.putNumber("LimelightX", x);
    //SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightID", id);

  }


  //Obtém eixo X em relação a AprilTag num campo 3D
  public double getX3d() {
    double x = LimelightHelpers.getBotPose3d("limelight").getX();
    return x;
  }

  //Obtém eixo Y em relação a AprilTag num campo 3D
  public double getY3d() {
    double y = LimelightHelpers.getBotPose3d("limelight").getY();
    return y;
  }

  public void setLedOn(boolean isOn){
    if(isOn) LimelightHelpers.setLEDMode_ForceOn("limelight");
    else     LimelightHelpers.setLEDMode_ForceOff("limelight");

  }
}
