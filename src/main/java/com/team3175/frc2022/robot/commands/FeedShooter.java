package com.team3175.frc2022.robot.commands;

import com.team3175.frc2022.robot.subsystems.Feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeedShooter extends CommandBase {

    private final Feeder m_feeder;
    private final double m_rpm;

    /**
     * 
     * Runs the feeder in RPM mode
     * 
     * @param feeder feeder instance
     * @param rpm setpoint for the motor in RPM
     * 
     */

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
