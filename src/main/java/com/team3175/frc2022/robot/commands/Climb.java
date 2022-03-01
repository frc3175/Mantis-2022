package com.team3175.frc2022.robot.commands;

import com.team3175.frc2022.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Climb extends SequentialCommandGroup {

    private Climber m_climber;
    private double m_upSetpoint;
    private double m_downSetpoint;
    private double m_climbSpeed;
    
    public Climb(Climber climber, double upSetpoint, double downSetpoint, double climbSpeed) {

        m_climber = climber;
        m_upSetpoint = upSetpoint;
        m_downSetpoint = downSetpoint;
        m_climbSpeed = climbSpeed;

        addRequirements(m_climber);

    }

    @Override
    public void initialize() {
        m_climber.resetEncoders();
    }

    @Override
    public void execute() {
        addCommands(new ClimbUp(m_climber, m_upSetpoint, m_climbSpeed),
                    new InstantCommand(() -> m_climber.overrideStop()),
                    new WaitCommand(1),
                    new ClimbDown(m_climber, m_downSetpoint, m_climbSpeed),
                    new InstantCommand(() -> m_climber.overrideStop())
        );
    }

}
