package com.team3175.frc2022.robot.commands;

import com.team3175.frc2022.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbUp extends CommandBase {

    private Climber m_climber;
    private double m_setpoint;
    private double m_speed;

    public ClimbUp(Climber climber, double setpoint, double speed) {

        m_climber = climber;
        m_setpoint = setpoint;
        m_speed = speed;

        addRequirements(m_climber);

    }

    @Override
    public void initialize() {
        m_climber.resetEncoders();
    }

    @Override
    public void execute() {
        m_climber.climbUp(m_setpoint, m_speed);
    }

    @Override
    public void end(boolean isFinished) {
        m_climber.overrideStop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
