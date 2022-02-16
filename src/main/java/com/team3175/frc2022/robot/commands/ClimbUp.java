package com.team3175.frc2022.robot.commands;

import com.team3175.frc2022.lib.math.Conversions;
import com.team3175.frc2022.robot.Constants;
import com.team3175.frc2022.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbUp extends CommandBase {

    private Climber m_climber;
    private double m_setpoint;
    private double m_speed;
    private Timer m_timer;

    public ClimbUp(Climber climber, double setpoint, double speed) {

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
    }

    @Override
    public void execute() {

        /*if(m_climber.getVelocity() > 0) {
            m_climber.unlockPneumatics();
        } else {
            m_climber.lockPneumatics();
        } */

        if(m_timer.get() > 0.25) {
            m_climber.climbUp(m_setpoint, m_speed);
        } else {
            m_climber.overrideStop();
        }


    }

    @Override
    public void end(boolean isFinished) {
        m_climber.overrideStop();
    }

    @Override
    public boolean isFinished() {
        if(m_climber.getClimberEncoder() > Conversions.climberInchesToEncoders(Constants.CLIMBER_UP_DISTANCE)) {
            return true;
        } else {
            return false;
        }
    }
    
}
