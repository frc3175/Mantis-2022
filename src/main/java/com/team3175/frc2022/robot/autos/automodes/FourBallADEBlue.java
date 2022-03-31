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

public class FourBallADEBlue extends SequentialCommandGroup {

    private Shooter m_shooter;
    private Feeder m_feeder;
    private Intake m_intake;
    private Actuators m_actuators;
    private SwerveDrivetrain m_drivetrain;
    private PathPlannerTrajectory m_intakeA;
    private PathPlannerTrajectory m_shootA;
    private PathPlannerTrajectory m_intakeDE;
    private PathPlannerTrajectory m_shootDE;

    public FourBallADEBlue(Shooter shooter, Feeder feeder, Intake intake, Actuators actuators, SwerveDrivetrain drivetrain) {

        m_shooter = shooter;
        m_feeder = feeder;
        m_drivetrain = drivetrain;
        m_intake = intake;
        m_actuators = actuators;

        m_intakeA = PathPlanner.loadPath("4BallADE-1-Blue", Constants.FOUR_BALL_MAX_SPEED, Constants.FOUR_BALL_MAX_ACCELERATION);
        m_shootA = PathPlanner.loadPath("4BallADE-2-Blue", Constants.FOUR_BALL_MAX_SPEED, Constants.FOUR_BALL_MAX_ACCELERATION);
        m_intakeDE = PathPlanner.loadPath("4BallADE-3-Blue", Constants.FOUR_BALL_MAX_SPEED, Constants.FOUR_BALL_MAX_ACCELERATION);
        m_shootDE = PathPlanner.loadPath("4BallADE-4-Blue", Constants.FOUR_BALL_MAX_SPEED, Constants.FOUR_BALL_MAX_ACCELERATION);

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

        PPSwerveControllerCommand m_intakeDECommand = 
            new PPSwerveControllerCommand(
            m_intakeDE, 
            m_drivetrain::getPose, 
            Constants.swerveKinematics, 
            m_translationController, 
            m_strafeController, 
            m_thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

        PPSwerveControllerCommand m_shootDECommand = 
            new PPSwerveControllerCommand(
            m_shootDE, 
            m_drivetrain::getPose, 
            Constants.swerveKinematics, 
            m_translationController, 
            m_strafeController, 
            m_thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

        AutonSpinUp m_spinUpA = new AutonSpinUp(m_shooter, Constants.SHOOTER_TARGET_RPM);
        AutonSpinUp m_spinUpDE = new AutonSpinUp(m_shooter, Constants.SHOOTER_TARGET_RPM);

        AutonShootAndFeed m_shootAndFeedA = new AutonShootAndFeed(m_shooter, m_feeder, Constants.FIVE_BALL_FEEDER_TICKS, Constants.SHOOTER_TARGET_RPM, Constants.FEEDER_PERCENT_OUTPUT);
        AutonShootAndFeed m_shootAndFeedDE = new AutonShootAndFeed(m_shooter, m_feeder, (Constants.FEEDER_TICKS * 2), Constants.SHOOTER_TARGET_RPM, Constants.FEEDER_PERCENT_OUTPUT);

        SetIntakeState m_intakeDeployA = new SetIntakeState(m_intake, m_actuators, "deploy", Constants.INTAKE_SPEED);
        SetIntakeState m_intakeDeployDE = new SetIntakeState(m_intake, m_actuators, "deploy", Constants.INTAKE_SPEED);

        SetIntakeState m_intakeRetractA = new SetIntakeState(m_intake, m_actuators, "retract", Constants.INTAKE_SPEED);
        SetIntakeState m_intakeRetractDE = new SetIntakeState(m_intake, m_actuators, "retract", Constants.INTAKE_SPEED);

        addCommands(new InstantCommand(() -> m_drivetrain.resetOdometry(new Pose2d(6.05, 5.16, Rotation2d.fromDegrees(137.49)))),
                    new ParallelCommandGroup(m_intakeACommand, m_intakeDeployA),
                    new ParallelCommandGroup(m_shootACommand, m_intakeRetractA, m_spinUpA),
                    new StopSwerve(m_drivetrain),
                    m_shootAndFeedA,
                    new ParallelCommandGroup(m_intakeDECommand, m_intakeDeployDE),
                    new ParallelCommandGroup(m_shootDECommand, m_intakeRetractDE, m_spinUpDE),
                    new StopSwerve(m_drivetrain),
                    m_shootAndFeedDE
                    );

    }

    
}
