package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral_Intake extends SubsystemBase{
    SparkMax intakeR, intakeL;
    SparkMaxConfig intakeRC, intakeLC;

    public Coral_Intake(){
        intakeR = new SparkMax(10, MotorType.kBrushless);
        intakeRC = new SparkMaxConfig();
        intakeL = new SparkMax(11, MotorType.kBrushless);
        intakeLC = new SparkMaxConfig();

        intakeRC.idleMode(IdleMode.kCoast);
        intakeLC.idleMode(IdleMode.kCoast);

        intakeRC.inverted(true);;
    }

    public void setIntake(double vel){
        intakeR.set(vel);
        intakeL.set(-vel);
    }
}
