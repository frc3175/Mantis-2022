package com.team3175.frc2022.robot.commands;

import com.team3175.frc2022.robot.subsystems.Feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeedShooter extends CommandBase {

    private final Feeder m_feeder;
    private final double m_rpm;
    
    public FeedShooter(Feeder feeder, double rpm) {

        m_feeder = feeder;
        m_rpm = rpm;
        
    }

    @Override
    public void initialize() {
        m_feeder.resetEncoders();
    }

    @Override
    public void execute() {
        m_feeder.feederRunVelocity(m_rpm);
    }





}
