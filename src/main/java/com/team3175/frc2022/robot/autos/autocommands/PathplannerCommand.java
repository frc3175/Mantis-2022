package com.team3175.frc2022.robot.autos.autocommands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.team3175.frc2022.robot.Constants;
import com.team3175.frc2022.robot.subsystems.SwerveDrivetrain;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PathplannerCommand extends CommandBase {

    private final SwerveDrivetrain m_swerveDrivetrain;
    private final PathPlannerTrajectory m_trajectory;
    private Timer m_timer = new Timer();
    private HolonomicDriveController hController;

    private final PIDController m_translationController;
    private final PIDController m_strafeController;
    private final ProfiledPIDController m_thetaController;

    public PathplannerCommand(SwerveDrivetrain drivetrain, PathPlannerTrajectory trajectory) {

        m_swerveDrivetrain = drivetrain;
        m_trajectory = trajectory;

        m_translationController = new PIDController(Constants.AUTO_P_X_CONTROLLER, 0, 0);
        m_strafeController = new PIDController(Constants.AUTO_P_Y_CONTROLLER, 0, 0);
        m_thetaController = new ProfiledPIDController(Constants.AUTO_P_THETA_CONTROLLER, 0, 0, 
                                                        Constants.THETA_CONTROLLER_CONSTRAINTS);
        //enable continuous output because we use falcons and thats what you do apparantly
        m_thetaController.enableContinuousInput(-Constants.MAX_ANGULAR_VELOCITY, Constants.MAX_ANGULAR_VELOCITY);

        hController = new HolonomicDriveController(m_translationController, m_strafeController, m_thetaController);

        addRequirements(m_swerveDrivetrain);

    }

    @Override
    public void initialize() {

        m_timer.reset();
        m_timer.start();

    }

    @Override
    public void execute() {

        double currentTime = m_timer.get();

        Trajectory.State desiredState = m_trajectory.sample(currentTime);
        Rotation2d desiredRotation = ((PathPlannerState)desiredState).holonomicRotation;

        ChassisSpeeds targetSpeeds = hController.calculate(m_swerveDrivetrain.getPose(), desiredState, desiredRotation);

        m_swerveDrivetrain.setModuleStates(Constants.swerveKinematics.toSwerveModuleStates(targetSpeeds));

    }

    @Override
    public boolean isFinished() {
        if(m_timer.get() > m_trajectory.getTotalTimeSeconds()) {
            return true;
        } else {
            return false;
        }
    }
    
}
