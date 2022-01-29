package com.team3175.frc2022.robot.commands;

import com.team3175.frc2022.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class StopShooter extends CommandBase {
    
    private final Shooter m_shooter;

    public StopShooter(Shooter shooter) {

        m_shooter = shooter;

        addRequirements(m_shooter);

    }

    @Override
    public void initialize() {

        m_shooter.resetEncoders();

    }

    @Override
    public void execute() {

        m_shooter.stopShooter();

    }

}
