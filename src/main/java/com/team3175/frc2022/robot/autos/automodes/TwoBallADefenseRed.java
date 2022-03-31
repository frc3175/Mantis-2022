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

public class TwoBallADefenseRed extends SequentialCommandGroup {

    private Shooter m_shooter;
    private Feeder m_feeder;
    private Intake m_intake;
    private Actuators m_actuators;
    private SwerveDrivetrain m_drivetrain;
    private PathPlannerTrajectory m_intakeA;
    private PathPlannerTrajectory m_shootA;
    private PathPlannerTrajectory m_intakeX;
    private PathPlannerTrajectory m_shootX;
    private PathPlannerTrajectory m_spin;

    public TwoBallADefenseRed(Shooter shooter, Feeder feeder, Intake intake, Actuators actuators, SwerveDrivetrain drivetrain) {

        m_shooter = shooter;
        m_feeder = feeder;
        m_drivetrain = drivetrain;
        m_intake = intake;
        m_actuators = actuators;

        m_intakeA = PathPlanner.loadPath("2BallA-Defense-1-Red", Constants.AUTO_MAX_SPEED, Constants.AUTO_MAX_ACCELERATION_MPS_SQUARED);
        m_shootA = PathPlanner.loadPath("2BallA-Defense-2-Red", Constants.AUTO_MAX_SPEED, Constants.AUTO_MAX_ACCELERATION_MPS_SQUARED);
        m_intakeX = PathPlanner.loadPath("2BallA-Defense-3-Red", Constants.AUTO_MAX_SPEED, Constants.AUTO_MAX_ACCELERATION_MPS_SQUARED);
        m_shootX = PathPlanner.loadPath("2BallA-Defense-4-Red", Constants.AUTO_MAX_SPEED, Constants.AUTO_MAX_ACCELERATION_MPS_SQUARED);
        m_spin = PathPlanner.loadPath("2BallA-Defense-5-Red", Constants.AUTO_MAX_SPEED, Constants.AUTO_MAX_ACCELERATION_MPS_SQUARED);

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

        PPSwerveControllerCommand m_intakeXCommand = 
            new PPSwerveControllerCommand(
            m_intakeX, 
            m_drivetrain::getPose, 
            Constants.swerveKinematics, 
            m_translationController, 
            m_strafeController, 
            m_thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

        PPSwerveControllerCommand m_shootXCommand = 
            new PPSwerveControllerCommand(
            m_shootX, 
            m_drivetrain::getPose, 
            Constants.swerveKinematics, 
            m_translationController, 
            m_strafeController, 
            m_thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

        PPSwerveControllerCommand m_spinCommand = 
            new PPSwerveControllerCommand(
            m_spin, 
            m_drivetrain::getPose, 
            Constants.swerveKinematics, 
            m_translationController, 
            m_strafeController, 
            m_thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

        AutonSpinUp m_spinUpA = new AutonSpinUp(m_shooter, Constants.SHOOTER_TARGET_RPM);
        AutonSpinUp m_spinUpX = new AutonSpinUp(m_shooter, Constants.SHOOTER_TARGET_RPM);

        AutonShootAndFeed m_shootAndFeedA = new AutonShootAndFeed(m_shooter, m_feeder, Constants.FEEDER_TICKS, Constants.SHOOTER_TARGET_RPM, Constants.FEEDER_PERCENT_OUTPUT);
        AutonShootAndFeed m_shootAndFeedX = new AutonShootAndFeed(m_shooter, m_feeder, (Constants.FEEDER_TICKS * 2), Constants.SHOOT_DEFENSIVE_BALL_SPEED, Constants.FEEDER_PERCENT_OUTPUT);

        SetIntakeState m_intakeDeployA = new SetIntakeState(m_intake, m_actuators, "deploy", Constants.INTAKE_SPEED);
        SetIntakeState m_intakeDeployX = new SetIntakeState(m_intake, m_actuators, "deploy", Constants.INTAKE_SPEED);

        SetIntakeState m_intakeRetractA = new SetIntakeState(m_intake, m_actuators, "retract", Constants.INTAKE_SPEED);
        SetIntakeState m_intakeRetractX = new SetIntakeState(m_intake, m_actuators, "retract", Constants.INTAKE_SPEED);

        addCommands(new InstantCommand(() -> m_drivetrain.resetOdometry(new Pose2d(6.05, 5.16, Rotation2d.fromDegrees(137.49)))),
                    new ParallelCommandGroup(m_intakeACommand, m_intakeDeployA),
                    new ParallelCommandGroup(m_shootACommand, m_intakeRetractA, m_spinUpA),
                    new StopSwerve(m_drivetrain),
                    m_shootAndFeedA,
                    new ParallelCommandGroup(m_intakeXCommand, m_intakeDeployX),
                    new ParallelCommandGroup(m_shootXCommand, m_intakeRetractX, m_spinUpX),
                    new StopSwerve(m_drivetrain),
                    m_shootAndFeedX,
                    m_spinCommand
                    );

    }

    
}
