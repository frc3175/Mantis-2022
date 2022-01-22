/* package com.team3175.frc2022.robot.autos.automodes;

import com.team3175.frc2022.robot.Constants;
import com.team3175.frc2022.robot.commands.SwerveDrive;
import com.team3175.frc2022.robot.subsystems.SwerveDrivetrain;
import com.pathplanner.lib.*;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class PathplannerTesting extends SequentialCommandGroup {
    
    private SwerveDrivetrain m_drivetrain;

    Trajectory m_trajectory = new Trajectory();
    PathPlannerState state;

    Trajectory.State targetState;
    Rotation2d targetRotation;

    public PathplannerTesting(SwerveDrivetrain drivetrain) {

        m_trajectory = PathPlanner.loadPath("Y-Loop", 8, 5);

        m_drivetrain = drivetrain;

        var m_translationController = new PIDController(Constants.AUTO_P_X_CONTROLLER, 0, 0);
        var m_strafeController = new PIDController(Constants.AUTO_P_Y_CONTROLLER, 0, 0);
        var m_thetaController = new ProfiledPIDController(Constants.AUTO_P_THETA_CONTROLLER, 0, 0, 
                                                        Constants.THETA_CONTROLLER_CONSTRAINTS);
        //enable continuous output because we use falcons and thats what you do apparantly
        m_thetaController.enableContinuousInput(-Constants.MAX_ANGULAR_VELOCITY, Constants.MAX_ANGULAR_VELOCITY);

        HolonomicDriveController hController = 
        new HolonomicDriveController(m_translationController, 
                                     m_strafeController, 
                                     m_thetaController);

        ChassisSpeeds targetSpeeds = hController.calculate(m_drivetrain.getPose(), 
                                                           targetState, 
                                                           targetRotation);

                                        

        
        addCommands( 
            new InstantCommand(() -> m_drivetrain.resetOdometry(m_trajectory.getInitialPose())),
            new SwerveDrive(m_drivetrain, driverController, driveAxis, strafeAxis, rotationAxis, fieldRelative, openLoop)
        );

    }   


} */
