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

public class FourBallBCDBlue extends SequentialCommandGroup {

    private Shooter m_shooter;
    private Feeder m_feeder;
    private Intake m_intake;
    private Actuators m_actuators;
    private SwerveDrivetrain m_drivetrain;
    private PathPlannerTrajectory m_intakeC;
    private PathPlannerTrajectory m_shootC;
    private PathPlannerTrajectory m_intakeBD;
    private PathPlannerTrajectory m_shootBD;

    public FourBallBCDBlue(Shooter shooter, Feeder feeder, Intake intake, Actuators actuators, SwerveDrivetrain drivetrain) {

        m_shooter = shooter;
        m_feeder = feeder;
        m_drivetrain = drivetrain;
        m_intake = intake;
        m_actuators = actuators;

        m_intakeC = PathPlanner.loadPath("4BallBCD-1-Blue", Constants.FOUR_BALL_MAX_SPEED, Constants.FOUR_BALL_MAX_ACCELERATION);
        m_shootC = PathPlanner.loadPath("4BallBCD-2-Blue", Constants.FOUR_BALL_MAX_SPEED, Constants.FOUR_BALL_MAX_ACCELERATION);
        m_intakeBD = PathPlanner.loadPath("4BallBCD-3-Blue", Constants.FOUR_BALL_MAX_SPEED, Constants.FOUR_BALL_MAX_ACCELERATION);
        m_shootBD = PathPlanner.loadPath("4BallBCD-4-Blue", Constants.FOUR_BALL_MAX_SPEED, Constants.FOUR_BALL_MAX_ACCELERATION);

        var m_translationController = new PIDController(Constants.AUTO_P_X_CONTROLLER, 0, 0);
        var m_strafeController = new PIDController(Constants.AUTO_P_Y_CONTROLLER, 0, 0);
        var m_thetaController = new ProfiledPIDController(Constants.AUTO_P_THETA_CONTROLLER, 0, 0, 
                                                        Constants.THETA_CONTROLLER_CONSTRAINTS);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand m_intakeCCommand = 
            new PPSwerveControllerCommand(
            m_intakeC, 
            m_drivetrain::getPose, 
            Constants.swerveKinematics, 
            m_translationController, 
            m_strafeController, 
            m_thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

        PPSwerveControllerCommand m_shootCCommand = 
            new PPSwerveControllerCommand(
            m_shootC, 
            m_drivetrain::getPose, 
            Constants.swerveKinematics, 
            m_translationController, 
            m_strafeController, 
            m_thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

        PPSwerveControllerCommand m_intakeBDCommand = 
            new PPSwerveControllerCommand(
            m_intakeBD, 
            m_drivetrain::getPose, 
            Constants.swerveKinematics, 
            m_translationController, 
            m_strafeController, 
            m_thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

        PPSwerveControllerCommand m_shootBDCommand = 
            new PPSwerveControllerCommand(
            m_shootBD, 
            m_drivetrain::getPose, 
            Constants.swerveKinematics, 
            m_translationController, 
            m_strafeController, 
            m_thetaController, 
            m_drivetrain::setModuleStates, 
            m_drivetrain);

        AutonSpinUp m_spinUpC = new AutonSpinUp(m_shooter, Constants.SHOOTER_TARGET_RPM);
        AutonSpinUp m_spinUpBD = new AutonSpinUp(m_shooter, Constants.SHOOTER_TARGET_RPM);

        AutonShootAndFeed m_shootAndFeedC = new AutonShootAndFeed(m_shooter, m_feeder, Constants.FIVE_BALL_FEEDER_TICKS, Constants.SHOOTER_TARGET_RPM, Constants.FEEDER_PERCENT_OUTPUT);
        AutonShootAndFeed m_shootAndFeedBD = new AutonShootAndFeed(m_shooter, m_feeder, (Constants.FEEDER_TICKS * 2), Constants.SHOOTER_TARGET_RPM, Constants.FEEDER_PERCENT_OUTPUT);

        SetIntakeState m_intakeDeployC = new SetIntakeState(m_intake, m_actuators, "deploy", Constants.INTAKE_SPEED);
        SetIntakeState m_intakeDeployBD = new SetIntakeState(m_intake, m_actuators, "deploy", Constants.INTAKE_SPEED);

        SetIntakeState m_intakeRetractC = new SetIntakeState(m_intake, m_actuators, "retract", Constants.INTAKE_SPEED);
        SetIntakeState m_intakeRetractBD = new SetIntakeState(m_intake, m_actuators, "retract", Constants.INTAKE_SPEED);

        addCommands(new InstantCommand(() -> m_drivetrain.resetOdometry(new Pose2d(7.56, 1.79, Rotation2d.fromDegrees(-88.09)))),
                    new ParallelCommandGroup(m_intakeCCommand, m_intakeDeployC),
                    new ParallelCommandGroup(m_shootCCommand, m_intakeRetractC, m_spinUpC),
                    new StopSwerve(m_drivetrain),
                    m_shootAndFeedC,
                    new ParallelCommandGroup(m_intakeBDCommand, m_intakeDeployBD),
                    new ParallelCommandGroup(m_shootBDCommand, m_intakeRetractBD, m_spinUpBD),
                    new StopSwerve(m_drivetrain),
                    m_shootAndFeedBD
                    );

    }

    
}
