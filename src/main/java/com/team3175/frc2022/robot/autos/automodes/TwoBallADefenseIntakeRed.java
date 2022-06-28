package com.team3175.frc2022.robot.autos.automodes;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.team3175.frc2022.robot.Constants;
import com.team3175.frc2022.robot.autos.autocommands.AutonShootAndFeed;
import com.team3175.frc2022.robot.autos.autocommands.AutonSpinUp;
import com.team3175.frc2022.robot.commands.SetIntakeState;
import com.team3175.frc2022.robot.commands.StopSwerve;
import com.team3175.frc2022.robot.subsystems.Actuators;
import com.team3175.frc2022.robot.subsystems.Feeder;
import com.team3175.frc2022.robot.subsystems.Intake;
import com.team3175.frc2022.robot.subsystems.Shooter;
import com.team3175.frc2022.robot.subsystems.SwerveDrivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TwoBallADefenseIntakeRed extends SequentialCommandGroup {

    private Shooter m_shooter;
    private Feeder m_feeder;
    private Intake m_intake;
    private Actuators m_actuators;
    private SwerveDrivetrain m_drivetrain;
    private PathPlannerTrajectory m_intakeA;
    private PathPlannerTrajectory m_shootA;
    private PathPlannerTrajectory m_intakeOpposite;
    private PathPlannerTrajectory m_driveToHangar;
    private Pose2d m_initialPose;

    public TwoBallADefenseIntakeRed(Shooter shooter, Feeder feeder, Intake intake, Actuators actuators, SwerveDrivetrain drivetrain) {

        m_shooter = shooter;
        m_feeder = feeder;
        m_drivetrain = drivetrain;
        m_intake = intake;
        m_actuators = actuators;
        m_initialPose = new Pose2d(6.05, 5.16, Rotation2d.fromDegrees(137.49));

        m_intakeA = PathPlanner.loadPath("2BallA-Defense-Intake-1-Red", Constants.AUTO_MAX_SPEED, Constants.AUTO_MAX_ACCELERATION_MPS_SQUARED);
        m_shootA = PathPlanner.loadPath("2BallA-Defense-Intake-2-Red", Constants.AUTO_MAX_SPEED, Constants.AUTO_MAX_ACCELERATION_MPS_SQUARED);
        m_intakeOpposite = PathPlanner.loadPath("2BallA-Defense-Intake-3-Red", Constants.AUTO_MAX_SPEED, Constants.AUTO_MAX_ACCELERATION_MPS_SQUARED);
        m_driveToHangar = PathPlanner.loadPath("2BallA-Defense-Intake-4-Red", Constants.AUTO_MAX_SPEED, Constants.AUTO_MAX_ACCELERATION_MPS_SQUARED);

        var m_translationController = new PIDController(Constants.AUTO_P_X_CONTROLLER, 0, 0);
        var m_strafeController = new PIDController(Constants.AUTO_P_Y_CONTROLLER, 0, 0);
        var m_thetaController = new ProfiledPIDController(Constants.AUTO_P_THETA_CONTROLLER, 0, 0, 
                                                        Constants.THETA_CONTROLLER_CONSTRAINTS);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand m_intakeACommand = 
            new PPSwerveControllerCommand(
            m_intakeA, 
            m_drivetrain::getPose, 
            Constants.swerveKinematics, 
            m_translationController, 
            m_strafeController, 
            m_thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

        PPSwerveControllerCommand m_shootACommand = 
            new PPSwerveControllerCommand(
            m_shootA, 
            m_drivetrain::getPose, 
            Constants.swerveKinematics, 
            m_translationController, 
            m_strafeController, 
            m_thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

        PPSwerveControllerCommand m_intakeOppositeCommand = 
            new PPSwerveControllerCommand(
            m_intakeOpposite, 
            m_drivetrain::getPose, 
            Constants.swerveKinematics, 
            m_translationController, 
            m_strafeController, 
            m_thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

        PPSwerveControllerCommand m_driveToHangarCommand = 
            new PPSwerveControllerCommand(
            m_driveToHangar, 
            m_drivetrain::getPose, 
            Constants.swerveKinematics, 
            m_translationController, 
            m_strafeController, 
            m_thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

        AutonSpinUp m_spinUp = new AutonSpinUp(m_shooter, Constants.SHOOTER_TARGET_RPM);

        AutonShootAndFeed m_shootAndFeed = new AutonShootAndFeed(m_shooter, m_feeder, Constants.FEEDER_TICKS, Constants.SHOOTER_TARGET_RPM, Constants.FEEDER_PERCENT_OUTPUT);

        SetIntakeState m_deployIntake = new SetIntakeState(m_intake, m_actuators, "deploy", Constants.INTAKE_SPEED);
        SetIntakeState m_deployIntakeSlow = new SetIntakeState(m_intake, m_actuators, "deploy", 0.1);
        SetIntakeState m_deployIntakeReverse = new SetIntakeState(m_intake, m_actuators, "deploy reverse", 0.9);

        addCommands(new InstantCommand(() -> m_drivetrain.resetOdometry(m_initialPose)),
                    new ParallelCommandGroup(m_intakeACommand, m_deployIntake),
                    new ParallelCommandGroup(m_shootACommand, m_spinUp),
                    new StopSwerve(m_drivetrain),
                    m_shootAndFeed,
                    new ParallelCommandGroup(m_intakeOppositeCommand, m_deployIntakeSlow),
                    m_driveToHangarCommand,
                    new StopSwerve(m_drivetrain),
                    m_deployIntakeReverse,
                    new InstantCommand(() -> m_drivetrain.setGyro(180.00))
                    );

    }

    
}
