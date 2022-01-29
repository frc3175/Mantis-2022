package com.team3175.frc2022.robot.commands;

import com.team3175.frc2022.robot.subsystems.Feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class StopFeeder extends CommandBase {
    
    private final Feeder m_shooter;

    public StopFeeder(Feeder shooter) {

        m_shooter = shooter;

        addRequirements(m_shooter);

    }

    @Override
    public void initialize() {

        //m_shooter.resetEncoders();

    }

    @Override
    public void execute() {

        m_shooter.stopFeeder();

    }

}
