package com.team3175.frc2022.robot.autos.autocommands;

import com.team3175.frc2022.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonSpinUp extends CommandBase {

    private Shooter m_shooter;
    private double m_rpm;
    private boolean isDone = false;

    /**
     * 
     * Spin up the shooter in auton, does not run the feeder
     * 
     * @param shooter shooter instance
     * @param rpm setpoint for the shooter in RPM
     * 
     */

    public AutonSpinUp(Shooter shooter, double rpm) {

        m_shooter = shooter;
        m_rpm = rpm;

        addRequirements(m_shooter);

    }

    @Override
    public void initialize() {

        m_shooter.resetEncoders();
        
    }

    @Override
    public void execute() {

        if(m_shooter.getLeftVelocityRPM() < m_rpm) {
            isDone = false;
        } else {
            isDone = true;
        }

        m_shooter.shoot(m_rpm);

    }

    @Override 
    public void end(boolean isFinished) {

        m_shooter.stopShooter();

    }

    @Override
    public boolean isFinished() {

        return isDone;

    }
    
}
