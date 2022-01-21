package com.team3175.frc2022.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team3175.frc2022.robot.CTREConfigs;
import com.team3175.frc2022.robot.Constants;

//
//Undecided if this should be state space or just have a regular setpoint
//At the moment, just uses a setpoint and PID but it is also only partially done.
//PID is super important too here!!!! We do NOT want a robot bouncing up and down
//
//Note: have an override button so that if the position control doesn't work perfectly
//we can still move up and down
//

public class Climber {

    private final TalonFX m_climberFalcon = new TalonFX(Constants.CLIMBER_FALCON);

    public Climber() {
        configureClimberMotor();
    }

    public void climbUp() {
        m_climberFalcon.set(ControlMode.Position, Constants.CLIMBER_UP_SETPOINT);
    }

    public void climbDown() {
        m_climberFalcon.set(ControlMode.Position, Constants.CLIMBER_DOWN_SETPOINT);
    }

    public void configureClimberMotor() {
        m_climberFalcon.configFactoryDefault();
        m_climberFalcon.configAllSettings(CTREConfigs.climberFXConfig);
        m_climberFalcon.setNeutralMode(Constants.CLIMBER_NEUTRAL_MODE);
        m_climberFalcon.setInverted(Constants.INVERT_CLIMBER);
    }
    
}
