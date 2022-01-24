package com.team3175.frc2022.robot.autos.automodes;

import com.team3175.frc2022.robot.Constants;
import com.team3175.frc2022.robot.subsystems.SwerveDrivetrain;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PPSwerveControllerAuto extends SequentialCommandGroup {

    private SwerveDrivetrain m_drivetrain;

    PathPlannerTrajectory m_trajectory;
    
    public PPSwerveControllerAuto(SwerveDrivetrain drivetrain) {

        m_drivetrain = drivetrain;

        m_trajectory = PathPlanner.loadPath("Y-Loop", Constants.AUTO_MAX_SPEED, Constants.AUTO_MAX_ACCELERATION_MPS_SQUARED);

        var m_translationController = new PIDController(Constants.AUTO_P_X_CONTROLLER, 0, 0);
        var m_strafeController = new PIDController(Constants.AUTO_P_Y_CONTROLLER, 0, 0);
        var m_thetaController = new ProfiledPIDController(Constants.AUTO_P_THETA_CONTROLLER, 0, 0, 
                                                        Constants.THETA_CONTROLLER_CONSTRAINTS);
        //enable continuous output because we use falcons and thats what you do apparantly
        m_thetaController.enableContinuousInput(-Constants.MAX_ANGULAR_VELOCITY, Constants.MAX_ANGULAR_VELOCITY);

        PPSwerveControllerCommand autoCommand = 
            new PPSwerveControllerCommand(
            m_trajectory, 
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
