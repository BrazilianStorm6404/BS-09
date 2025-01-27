package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.SwerveAutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class TestCmd extends SequentialCommandGroup{

    Limelight limelight;
    TrajectoryConfig config;
    
    public TestCmd(Swerve sb_swerve){
        limelight = new Limelight();
        AutoBlueLimeLeftCmd autoBlueCmd1 = new AutoBlueLimeLeftCmd(sb_swerve,
                                            () -> (-limelight.getX()),
                                            () -> (limelight.getX3d()), 
                                            () -> (limelight.getY3d()),
                                            true, true, true, false);
        AutoBlueLimeLeftCmd autoBlueCmd2 = new AutoBlueLimeLeftCmd(sb_swerve,
                                            () -> (-limelight.getX()),
                                            () -> (limelight.getX3d()), 
                                            () -> (limelight.getY3d()),
                                            true, true, false, true);
        sb_swerve.zeroHeading();
        sb_swerve.setGyroOffset(0);

        config = new TrajectoryConfig(.5, 0.4)
                     .setKinematics(SwerveConstants.kinematics);

        Trajectory botTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
                List.of(new Translation2d(0, 0.006)),
            new Pose2d(-0.43, -0.001, new Rotation2d(Math.toRadians(22))),
            config
        );
            
        ProfiledPIDController thetaController = new ProfiledPIDController(0.04, 0, 0, SwerveAutoConstants.kThetaControllerConstraints);
        thetaController.disableContinuousInput();

        PIDController xPID = new PIDController(0.1, 0, 0);
        PIDController yPID = new PIDController(0.1, 0, 0);

        SwerveControllerCommand Step1 = new SwerveControllerCommand(
            botTrajectory, 
            sb_swerve::getPose,
            SwerveConstants.kinematics,
            xPID, 
            yPID, 
            thetaController,
            sb_swerve::setModuleStates,
            sb_swerve
        );

        addCommands(
            //autoBlueCmd1, 
            new InstantCommand(() -> sb_swerve.resetOdometry(botTrajectory.getInitialPose())),
            Step1,
            autoBlueCmd2,
            new InstantCommand(() -> sb_swerve.stopModules())
        );
          
    }
}
