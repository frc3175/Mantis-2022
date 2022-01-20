package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.CTREConfigs;
import frc.robot.Constants;

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
