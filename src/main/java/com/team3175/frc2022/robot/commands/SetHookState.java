package com.team3175.frc2022.robot.commands;

import com.team3175.frc2022.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetHookState extends CommandBase {

    private final Climber m_climber;
    private final String m_state;
    boolean isDone = false;

    /**
     * 
     * Constructor for the SetHookState command
     * 
     * @param climber climber instance
     * @param state state to set the hooks, "up" or "down"
     * 
     */

    public SetHookState(Climber climber, String state) {

        m_climber = climber;
        m_state = state;

        addRequirements(m_climber);

    }

    @Override
    public void initialize() {

        switch(m_state) {

            case "up" :
                m_climber.passiveHooksRelease();
                break;
            case "down" :
                m_climber.passiveHooksLock();
                break;

        }

        isDone = true;

    }

    @Override
    public boolean isFinished() {

        return isDone;
        
    }
    
}
