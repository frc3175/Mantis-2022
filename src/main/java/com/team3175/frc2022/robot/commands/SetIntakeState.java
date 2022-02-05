package com.team3175.frc2022.robot.commands;

import com.team3175.frc2022.robot.subsystems.Actuators;
import com.team3175.frc2022.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetIntakeState extends CommandBase {

    private Intake m_intake;
    private Actuators m_actuators;
    private String m_state;
    private double m_power;
    private boolean isDone = false;

    public SetIntakeState(Intake intake, Actuators actuators, String state, double power) {

        m_intake = intake;
        m_actuators = actuators;
        m_state = state;
        m_power = power;

        addRequirements(m_intake, m_actuators);

    }

    @Override
    public void initialize() {

        switch(m_state) {

            case "retract":
                m_actuators.actuateBack();
                m_intake.stopIntake();
                break;

            case "deploy":
                m_actuators.actuate();
                m_intake.intakeCargo(m_power);
                break;

        }

        isDone = true;

    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
    
}