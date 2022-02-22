package com.team3175.frc2022.robot.commands;

import com.team3175.frc2022.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OverrideClimbDown extends CommandBase {
    
    private Climber m_climber;
    private double m_speed;
    private Timer m_timer;

    public OverrideClimbDown(Climber climber, double speed) {

        m_climber = climber;
        m_speed = speed;
        m_timer = new Timer();

        addRequirements(m_climber);

    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_climber.unlockPneumatics();
    }

    @Override
    public void execute() {

        if(m_timer.get() > 0.25) {
            m_climber.overrideDown(m_speed);
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
    
}
