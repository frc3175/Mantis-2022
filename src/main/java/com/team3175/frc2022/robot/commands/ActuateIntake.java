package com.team3175.frc2022.robot.commands;

import com.team3175.frc2022.robot.subsystems.Actuaters;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ActuateIntake extends CommandBase{
    
    private  Actuaters m_actuaters;

    public ActuateIntake(Actuaters actuaters){

        m_actuaters = actuaters;

        addRequirements(m_actuaters);
    }

    @Override
    public void execute(){

        m_actuaters.actuate();
    }
}
