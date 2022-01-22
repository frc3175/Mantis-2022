package com.team3175.frc2022.robot.autos.automodes;

import java.io.IOException;
import java.nio.file.Path;

import com.team3175.frc2022.robot.Constants;
import com.team3175.frc2022.robot.RobotContainer;
import com.team3175.frc2022.robot.subsystems.SwerveDrivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class PathweaverTest extends SequentialCommandGroup {

    private SwerveDrivetrain m_drivetrain;

    Trajectory m_trajectory = new Trajectory();
    String pathJSON = "paths/TestPath.wpilib.json";

    public PathweaverTest(SwerveDrivetrain drivetrain) {

        m_drivetrain = drivetrain;

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(pathJSON);
            m_trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
          } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + pathJSON, ex.getStackTrace());
        } 

        var m_translationController = new PIDController(Constants.AUTO_P_X_CONTROLLER, 0, 0);
        var m_strafeController = new PIDController(Constants.AUTO_P_Y_CONTROLLER, 0, 0);
        var m_thetaController = new ProfiledPIDController(Constants.AUTO_P_THETA_CONTROLLER, 0, 0, 
                                                        Constants.THETA_CONTROLLER_CONSTRAINTS);
        //enable continuous output because we use falcons and thats what you do apparantly
        m_thetaController.enableContinuousInput(-Constants.MAX_ANGULAR_VELOCITY, Constants.MAX_ANGULAR_VELOCITY);

        SwerveControllerCommand autoCommand = 
        new SwerveControllerCommand(m_trajectory, 
                                    m_drivetrain::getPose, 
                                    Constants.swerveKinematics, 
                                    m_translationController, 
                                    m_strafeController, 
                                    m_thetaController, 
                                    m_drivetrain::setModuleStates, 
                                    m_drivetrain);
        
        addCommands( 
            new InstantCommand(() -> m_drivetrain.resetOdometry(m_trajectory.getInitialPose())),
            autoCommand
        );

    }   

}
