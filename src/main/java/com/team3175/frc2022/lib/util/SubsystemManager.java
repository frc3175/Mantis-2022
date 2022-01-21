/*
 * Ideally, this class will eventually utilize method overloading to just be able 
 * to create one object per subsystem. At the moment, a SubsystemManager object
 * must be created for each motor/actuator in the subsystem.
 */

package com.team3175.frc2022.lib.util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SubsystemManager {

    private final TalonFX m_motor;
    private final String m_name;

    /**
     * 
     * Subsystem Manager is a class to configure a motor using a CTRE config 
     * object and push all diagnostics to SmartDashboard. Create an instance 
     * of this class for each motor/actuator in a subsystem.
     * 
     * @param motor the motor to be configured
     * @param config the configuration created in CTREConfigs.java
     * @param isInverted if the motor is inverted
     * @param neutralMode brake mode or coast mode
     * @param name name with which to refer to the motor
     * 
     */
    public SubsystemManager(TalonFX motor, TalonFXConfiguration config, boolean isInverted, NeutralMode neutralMode, String name) {

        m_motor = motor;
        m_name = name;

        motor.configFactoryDefault();
        motor.configAllSettings(config);
        motor.setInverted(isInverted);
        motor.setNeutralMode(neutralMode);

    }

    public boolean isMotorAlive() {

        if(m_motor.getMotorOutputPercent() > 0) {
            return true;
        } else {
            return false;
        }

    }

    public double getTemp() {

        return m_motor.getTemperature();

    }

    public double getVelocity() {

        return m_motor.getSelectedSensorVelocity();

    }

    public void manageSubsystem() {

        String aliveKey = m_name + "isAlive: ";
        String tempKey = m_name + "temp: ";
        String velocityKey = m_name + "velocity: ";

        SmartDashboard.putBoolean(aliveKey, isMotorAlive());
        SmartDashboard.putNumber(tempKey, getTemp());
        SmartDashboard.putNumber(velocityKey, getVelocity());

    }
    
}
