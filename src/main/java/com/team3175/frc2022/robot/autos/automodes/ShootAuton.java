package com.team3175.frc2022.robot.autos.automodes;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.team3175.frc2022.robot.Constants;
import com.team3175.frc2022.robot.autos.autocommands.AutonShootAndFeed;
import com.team3175.frc2022.robot.autos.autocommands.AutonSpinUp;
import com.team3175.frc2022.robot.subsystems.Actuators;
import com.team3175.frc2022.robot.subsystems.Feeder;
import com.team3175.frc2022.robot.subsystems.Intake;
import com.team3175.frc2022.robot.subsystems.Shooter;
import com.team3175.frc2022.robot.subsystems.SwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootAuton extends SequentialCommandGroup {

    private Shooter m_shooter;
    private Feeder m_feeder;
    private Intake m_intake;
    private Actuators m_actuators;
    private SwerveDrivetrain m_drivetrain;
    private PathPlannerTrajectory m_trajectory;
    private PathPlannerTrajectory m_trajectory2;

    public ShootAuton(Shooter shooter, Feeder feeder, Intake intake, Actuators actuators, SwerveDrivetrain drivetrain) {

        m_shooter = shooter;
        m_feeder = feeder;
        m_drivetrain = drivetrain;
        m_intake = intake;
        m_actuators = actuators;

        AutonSpinUp m_spinUp1 = new AutonSpinUp(m_shooter, Constants.SHOOTER_TARGET_RPM);

        AutonShootAndFeed m_shootAndFeed1 = new AutonShootAndFeed(m_shooter, m_feeder, Constants.FIVE_BALL_FEEDER_TICKS, Constants.SHOOTER_TARGET_RPM, Constants.FEEDER_PERCENT_OUTPUT);

        addCommands(m_spinUp1,
                    m_shootAndFeed1
                    );

    }

    
}