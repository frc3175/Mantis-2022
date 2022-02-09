package com.team3175.frc2022.robot.commands;

import com.team3175.frc2022.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbDown extends CommandBase {

    private final Climber m_climber;
    private final double m_setpoint;
    private final double m_speed;

    public ClimbDown(Climber climber, double setpoint, double speed) {

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
        m_climber.climbDown(m_setpoint, m_speed);
        SmartDashboard.putNumber("climber encoder", m_climber.getClimberEncoder());
    }

    @Override
    public void end(boolean isFinished) {
        m_climber.overrideStop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
