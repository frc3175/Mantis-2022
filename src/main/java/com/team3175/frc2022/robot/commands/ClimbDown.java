package com.team3175.frc2022.robot.commands;

import com.team3175.frc2022.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbDown extends CommandBase {

    private final Climber m_climber;
    private final double m_setpoint;
    private final double m_speed;
    private Timer m_timer;

    public ClimbDown(Climber climber, double setpoint, double speed) {

        m_climber = climber;
        m_setpoint = setpoint;
        m_speed = speed;
        m_timer = new Timer();

        addRequirements(m_climber);

    }

    @Override
    public void initialize() {

        m_climber.resetEncoders();
        m_timer.reset();
        m_timer.start();
        m_climber.unlockPneumatics();
        m_climber.setDone(false);

    }

    @Override
    public void execute() {

        if(m_timer.get() > 0.25) {
            m_climber.climbDown(m_setpoint, m_speed);
        } else {
            m_climber.overrideStop();
        }

    }

    @Override
    public void end(boolean isFinished) {

        m_timer.reset();
        m_timer.start();
        while(m_timer.get() < 0.25) {
            //donothing
        }
        m_climber.lockPneumatics();

    }

    @Override
    public boolean isFinished() {
        return m_climber.isDone();
    }
    
}
