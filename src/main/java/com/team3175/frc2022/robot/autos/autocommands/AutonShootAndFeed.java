package com.team3175.frc2022.robot.autos.autocommands;

import com.team3175.frc2022.robot.subsystems.Feeder;
import com.team3175.frc2022.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonShootAndFeed extends CommandBase {

    private Shooter m_shooter;
    private Feeder m_feeder;
    private double m_ticks;
    private double m_feederPower;
    private double m_setpoint;
    private boolean isDone = false;

    public AutonShootAndFeed(Shooter shooter, Feeder feeder, double ticks, double setpoint, double feederPower) {

        m_shooter = shooter;
        m_feeder = feeder;
        m_ticks = ticks;
        m_setpoint = setpoint;
        m_feederPower = feederPower;

        addRequirements(m_shooter);

    }

    @Override
    public void initialize() {

        m_shooter.resetEncoders();

    }

    @Override
    public void execute() {

        boolean isLeftAtSetpoint = m_shooter.leftFalconAtSetpoint((m_setpoint - 100));
        boolean isRightAtSetpoint = m_shooter.rightFalconAtSetpoint((m_setpoint - 100));

        m_shooter.shoot(m_setpoint);

        if(isLeftAtSetpoint && isRightAtSetpoint) {
            if(m_feeder.getEncoder() < m_ticks) {
                m_feeder.feederRunPercentOutput(m_feederPower);
                m_shooter.shoot(m_setpoint);
            } else {
                isDone = true;
            }
        } else {
            m_shooter.shoot(m_setpoint);
        }

    }

    @Override 
    public void end(boolean isFinished) {

        m_feeder.stopFeeder();
        m_shooter.stopShooter();

    }

    @Override 
    public boolean isFinished() {

        return isDone;

    }
    
}
