package com.team3175.frc2022.robot.autos.autocommands;

import com.team3175.frc2022.robot.subsystems.Actuators;
import com.team3175.frc2022.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonIntake extends CommandBase {

    private Intake m_intake;
    private double m_ticks;
    private double m_speed;
    private Actuators m_actuators;
    private boolean isDone = false;

    public AutonIntake(Intake intake, Actuators actuators, double ticks, double speed) {

        m_intake = intake;
        m_ticks = ticks;
        m_speed = speed;
        m_actuators = actuators;

        addRequirements(m_intake, m_actuators);

    }

    @Override
    public void initialize() {

        m_actuators.actuate();
        m_intake.resetEncoders();
        
    }

    @Override
    public void execute() {

        double encoderPos = m_intake.getEncoders();

        if(encoderPos < m_ticks) {
            isDone = false;
        } else {
            isDone = true;
        }

        m_intake.intakeCargo(m_speed);

    }

    @Override 
    public void end(boolean isFinished) {

        m_intake.stopIntake();
        m_actuators.actuateBack();

    }

    @Override
    public boolean isFinished() {

        return isDone;

    }
    
}
