package com.team3175.frc2022.robot.commands;

import com.team3175.frc2022.robot.subsystems.Actuators;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ActuateBack extends CommandBase{
    
    private  Actuators m_actuators;

    public ActuateBack(Actuators actuators){

        m_actuators = actuators;

        addRequirements(m_actuators);
    }

    @Override
    public void execute(){

        m_actuators.actuateBack();
    }
}

