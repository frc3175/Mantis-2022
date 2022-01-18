package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.CTREConfigs;
import frc.robot.Constants;

public class Hopper {

    private TalonFX m_hopperFalcon = new TalonFX(Constants.HOPPER);

    public Hopper() {
        configHopperMotor();
    }

    private void configHopperMotor() {
        m_hopperFalcon.configFactoryDefault();
        m_hopperFalcon.configAllSettings(CTREConfigs.hopperFXConfig);
        m_hopperFalcon.setInverted(Constants.INVERT_HOPPER);
        m_hopperFalcon.setNeutralMode(Constants.HOPPER_NEUTRAL_MODE);
    }

    
    
}
