package com.team3175.frc2022.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team3175.frc2022.lib.math.Conversions;
import com.team3175.frc2022.robot.CTREConfigs;
import com.team3175.frc2022.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

    private TalonFX m_feederFalcon;
    //private SubsystemManager m_manager;

    public Feeder() {

        /*m_manager = new SubsystemManager(m_feederFalcon, 
                                         CTREConfigs.feederFXConfig, 
                                         Constants.INVERT_FEEDER, 
                                         Constants.FEEDER_NEUTRAL_MODE, 
                                         "feeder"); */

        m_feederFalcon = new TalonFX(Constants.FEEDER_FALCON);
        configFeederMotor();
        
    }

    /**
     * 
     * Run the feeder in PercentOutput mode
     * 
     * @param power PercentOutput to run the feeder
     * 
     */

    public void feederRunPercentOutput(double power) {
        m_feederFalcon.set(ControlMode.PercentOutput, power);
    }

    /**
     * 
     * Run the feeder in velocity mode
     * 
     * @param rpm setpoint of the feeder in revolutions per minute
     */

    public void feederRunVelocity(double rpm) {
        double falconUnits = Conversions.RPMToFalcon(rpm, 1.0);
        m_feederFalcon.set(ControlMode.Velocity, falconUnits);
    }

    /**
     * 
     * Sets feeder to PercentOutput mode and sets power to 0
     * 
     */

    public void stopFeeder() {
        m_feederFalcon.set(ControlMode.PercentOutput, 0);
    }

    /**
     * 
     * Set feeder encoder to absolute
     * 
     */

    public void resetEncoders() {
        m_feederFalcon.setSelectedSensorPosition(0);
    }

    /**
     * 
     * @return Position of the feeder encoder
     * 
     */

    public double getEncoderPos() {
        return m_feederFalcon.getSelectedSensorPosition();
    }

    /*public SubsystemManager getManager() {
        return m_manager;
    } */

    public void configFeederMotor() {
        m_feederFalcon.configFactoryDefault();
        m_feederFalcon.configAllSettings(CTREConfigs.feederFXConfig);
        m_feederFalcon.setInverted(Constants.INVERT_FEEDER);
        m_feederFalcon.setNeutralMode(Constants.FEEDER_NEUTRAL_MODE);
    }

    
    
}
