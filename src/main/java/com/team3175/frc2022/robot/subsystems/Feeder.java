package com.team3175.frc2022.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team3175.frc2022.lib.math.Conversions;
import com.team3175.frc2022.robot.CTREConfigs;
import com.team3175.frc2022.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

    private TalonFX m_feederFalcon;

    public Feeder() {

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
     * 
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

    public double getEncoder() {
        return m_feederFalcon.getSelectedSensorPosition();
    }

    /**
     * 
     * Configure feeder motor to the motor set in CTREConfig
     * 
     */

    public void configFeederMotor() {
        m_feederFalcon.configFactoryDefault();
        m_feederFalcon.configAllSettings(CTREConfigs.feederFXConfig);
        m_feederFalcon.setInverted(Constants.INVERT_FEEDER);
        m_feederFalcon.setNeutralMode(Constants.FEEDER_NEUTRAL_MODE);
    }

    /**
     * 
     * @return falcon temperature
     * 
     */

    public double getTemp() {
        return m_feederFalcon.getTemperature();
    }

    /**
     * 
     * @return current draw of the falcon (amps)
     * 
     */

    public double getCurrent() {
        return m_feederFalcon.getSupplyCurrent();
    }

    /**
     * 
     * @return if the falcon is drawing any voltage
     * 
     */

    public boolean isAlive() {
        return (m_feederFalcon.getBusVoltage() != 0.0);
    }

    /**
     * 
     * @return velocity of the motor in native falcon units
     * 
     */
    
    public double getVelocity() {
        return m_feederFalcon.getSelectedSensorVelocity();
    }

    
    
}
