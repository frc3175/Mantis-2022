package com.team3175.frc2022.robot.autos.automodes;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.team3175.frc2022.robot.Constants;
import com.team3175.frc2022.robot.autos.autocommands.AutonShootAndFeed;
import com.team3175.frc2022.robot.autos.autocommands.AutonSpinUp;
import com.team3175.frc2022.robot.subsystems.Feeder;
import com.team3175.frc2022.robot.subsystems.Shooter;
import com.team3175.frc2022.robot.subsystems.SwerveDrivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OneBallAuto extends SequentialCommandGroup {

    private Shooter m_shooter;
    private Feeder m_feeder;
    private SwerveDrivetrain m_drivetrain;
    private PathPlannerTrajectory m_driveBack;

    public OneBallAuto(Shooter shooter, Feeder feeder, SwerveDrivetrain drivetrain) {

        m_shooter = shooter;
        m_feeder = feeder;
        m_drivetrain = drivetrain;

        m_driveBack = PathPlanner.loadPath("tbd", Constants.AUTO_MAX_SPEED, Constants.AUTO_MAX_ACCELERATION_MPS_SQUARED);

        var m_translationController = new PIDController(Constants.AUTO_P_X_CONTROLLER, 0, 0);
        var m_strafeController = new PIDController(Constants.AUTO_P_Y_CONTROLLER, 0, 0);
        var m_thetaController = new ProfiledPIDController(Constants.AUTO_P_THETA_CONTROLLER, 0, 0, 
                                                        Constants.THETA_CONTROLLER_CONSTRAINTS);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand m_driveBackCommand = 
            new PPSwerveControllerCommand(
            m_driveBack, 
            m_drivetrain::getPose, 
            Constants.swerveKinematics, 
            m_translationController, 
            m_strafeController, 
            m_thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

        AutonSpinUp m_spinUp = new AutonSpinUp(m_shooter, Constants.SHOOTER_TARGET_RPM);

        AutonShootAndFeed m_shootAndFeed = new AutonShootAndFeed(m_shooter, m_feeder, Constants.SHOOT_TICKS, Constants.SHOOTER_TARGET_RPM, Constants.FEEDER_PERCENT_OUTPUT);

        addCommands(new InstantCommand(() -> m_drivetrain.resetOdometry(m_driveBack.getInitialPose())),
                    m_spinUp,
                    m_shootAndFeed,
                    m_driveBackCommand);

    }

    
}
